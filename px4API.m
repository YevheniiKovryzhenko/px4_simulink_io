classdef px4API < handle
    properties
        % User-defined configuration parameters
        MatlabProjectRoot = fullfile('~', 'GitHub', 'px4_simulink_io'); % Path to your main PX4 repository
        PX4Root = fullfile('~', 'PX4', 'v1.17.0'); % Path to your main PX4 repository
        PX4ModuleName = 'simulink_io';            % Target PX4 module folder name
        AllowedExtensions = {'.cpp', '.h'};       % File filter types
        LocalGeneratedDir = 'generatedcode';          % Local folder for generated artifacts before export
    end

    properties (Access = private)
        % Internal derived paths
        ResolvedExternalDir = '';
        StubHeaderName = 'px4_simulink_api.h';
        OrbCache = struct();
        OrbCacheLoaded = false;
        OrbCacheFile = 'orb_id_cache.json';
    end

    methods (Access = private)
        function resolvedPath = resolveAbsolutePath(~, pathStr)
            % Helper: Convert tilde, relative, and other path formats to absolute paths
            resolvedPath = pathStr;
            if startsWith(resolvedPath, '~/') || strcmp(resolvedPath, '~')
                resolvedPath = fullfile(getenv('HOME'), resolvedPath(2:end));
            elseif startsWith(resolvedPath, './') || startsWith(resolvedPath, '../')
                % Convert relative local terminal dot paths to true absolute paths
                resolvedPath = char(java.io.File(resolvedPath).getCanonicalPath());
            end
        end
    end

    methods
        function obj = px4API()
            % Enforce absolute resolution for the user-supplied project root folder
            obj.MatlabProjectRoot = obj.resolveAbsolutePath(obj.MatlabProjectRoot);

            % Enforce absolute resolution for the user-supplied PX4 root folder
            obj.PX4Root = obj.resolveAbsolutePath(obj.PX4Root);

            % Compute the exact target folder using native PX4 internal layout
            obj.ResolvedExternalDir = fullfile(obj.PX4Root, 'src', 'modules', ...
                                               obj.PX4ModuleName, 'generated_code');

            % Default local artifact/cache directory is a project-local folder
            if isempty(obj.LocalGeneratedDir)
                obj.LocalGeneratedDir = 'generatedcode';
            end
            obj.LocalGeneratedDir = fullfile(obj.MatlabProjectRoot, obj.LocalGeneratedDir);
            if ~exist(obj.LocalGeneratedDir, 'dir')
                mkdir(obj.LocalGeneratedDir);
            end
            if isempty(strfind(path, obj.LocalGeneratedDir))
                addpath(obj.LocalGeneratedDir);
            end

            % --- HANDS-FREE AUTOMATION ON INITIATION (skippable) ---
            fprintf('\n--- [px4API] Initializing & Scanning PX4 Message Directory ---\n');
            
            % ALWAYS regenerate bus objects in workspace (they don't persist across clears)
            fprintf('Ensuring Simulink bus types are in workspace...\n');
            obj.regenerateBusesInWorkspace();
            
            % Only regenerate files if needed
            if obj.needsGeneration()
                % Prepare all generated artifacts locally (header, source, glue, cache)
                obj.prepareLocalGeneratedArtifacts();
            else
                fprintf('✓ Generated artifacts present and up-to-date; skipping file regeneration.\n');
            end
        end

        function prepareLocalGeneratedArtifacts(obj)
            % Generate all local artifacts (header, source, glue) into the MATLAB project folder
            fprintf('\n--- [px4API] Preparing local generated artifacts in %s ---\n', obj.LocalGeneratedDir);
            
            % Rebuild/populate ORB cache FIRST - must be done before generateOmnipotentCppGlue
            % This pre-populates the cache so that 201 findOrbIdForTopic() calls don't timeout
            fprintf('Pre-loading ORB cache from PX4 source...\n');
            obj.rebuildOrbCacheFromDisk();
            
            % Generate busses and stub header/source locally
            obj.generateAllBussesAndHeaders(obj.LocalGeneratedDir);

            % Generate omnipotent glue locally (in MatlabProjectRoot)
            try
                obj.generateOmnipotentCppGlue(obj.LocalGeneratedDir);
            catch ME
                fprintf('! Warning: generateOmnipotentCppGlue failed: %s\n', ME.message);
            end
        end

        function regenerateBusesInWorkspace(obj)
            % Regenerate Simulink bus objects and assign to base workspace
            % This is ALWAYS called (even on cache hits) because bus objects are in-memory
            % and don't persist across workspace clears
            % Does NOT regenerate files - only creates in-memory MATLAB objects
            
            msgDir = fullfile(obj.PX4Root, 'msg');
            if ~exist(msgDir, 'dir')
                fprintf('! Warning: Could not find PX4 msg directory, skipping bus generation\n');
                return;
            end

            msgFiles = dir(fullfile(msgDir, '*.msg'));
            if isempty(msgFiles)
                fprintf('! Warning: No .msg files found, skipping bus generation\n');
                return;
            end

            fprintf('Generating Simulink bus types from %d message profiles...\n', length(msgFiles));
            
            busCount = 0;
            for i = 1:length(msgFiles)
                [~, camelName, ~] = fileparts(msgFiles(i).name);
                topicName = px4API.camelCaseToSnakeCase(camelName);

                try
                    [~, busObj, ~] = obj.generateBusFromMsg(camelName, topicName);
                    if ~isempty(busObj)
                        nativeStructName = [topicName, '_s'];
                        assignin('base', nativeStructName, busObj);
                        busCount = busCount + 1;
                    end
                catch ME
                    % Silently skip errors during bus generation
                end
            end
            fprintf('✓ Regenerated %d Simulink bus types in workspace\n', busCount);
        end

        function populateCacheNow(obj)
            % Public helper to force-populate the ORB cache into LocalGeneratedDir
            fprintf('--- [px4API] Populating ORB cache into %s ---\n', obj.LocalGeneratedDir);
            obj.rebuildOrbCacheFromBuild();
            fprintf('✓ ORB cache populated.\n');
        end

        function needed = needsGeneration(obj)
            % Determine whether generation is necessary.
            % Strategy: if the stub header and source exist and are newer than
            % the newest .msg file in PX4 msg/, we can skip regeneration.
            needed = true;

            hdrPath = fullfile(obj.LocalGeneratedDir, obj.StubHeaderName);
            srcPath = fullfile(obj.LocalGeneratedDir, 'px4_simulink_api.cpp');
            gluePath = fullfile(obj.LocalGeneratedDir, 'simulink_io_glue.cpp');

            if ~(exist(hdrPath, 'file') == 2 && exist(srcPath, 'file') == 2 && exist(gluePath, 'file') == 2)
                return;
            end

            msgDir = fullfile(obj.PX4Root, 'msg');
            if ~exist(msgDir, 'dir')
                return; % conservative: if we can't find msg dir, require generation
            end

            % Find newest msg file modification time
            msgFiles = dir(fullfile(msgDir, '*.msg'));
            if isempty(msgFiles)
                needed = false;
                return;
            end
            newestMsg = max([msgFiles(:).datenum]);

            hdrInfo = dir(hdrPath);
            srcInfo = dir(srcPath);
            glueInfo = dir(gluePath);
            hdrTime = hdrInfo.datenum;
            srcTime = srcInfo.datenum;
            glueTime = glueInfo.datenum;

            % if all generated files are newer or equal to newest msg, skip
            needed = ~(hdrTime >= newestMsg && srcTime >= newestMsg && glueTime >= newestMsg);
        end

        function orbId = findOrbIdForTopic(obj, snakeName)
            % Try to locate the generated PX4 topic header and extract the first ORB_DECLARE token.
            % Check in-memory / on-disk cache first to avoid repeated filesystem scans.
            orbId = snakeName; % fallback

            % lazy-load cache
            if ~obj.OrbCacheLoaded
                obj.loadOrbCache();
            end
            if isfield(obj.OrbCache, snakeName)
                orbId = obj.OrbCache.(snakeName);
                return;
            end

            % First try: look for uORB/topics/<snakeName>.h inside PX4 build folders
            findCmd = sprintf('find %s -path "*/uORB/topics/%s.h" -print -quit 2>/dev/null', obj.PX4Root, snakeName);
            [status, out] = system(findCmd);
            hdrPath = strtrim(out);
            if status ~= 0 || isempty(hdrPath)
                % Fallback: search for any file named <snakeName>.h
                findCmd2 = sprintf('find %s -name "%s.h" -print -quit 2>/dev/null', obj.PX4Root, snakeName);
                [status2, out2] = system(findCmd2);
                hdrPath = strtrim(out2);
                if status2 ~= 0 || isempty(hdrPath)
                    return; % give up, use fallback
                end
            end

            % Read the header and extract ORB_DECLARE(...) occurrence
            fid = fopen(hdrPath, 'r');
            if fid == -1
                return;
            end
            txt = textscan(fid, '%s', 'Delimiter', '\n');
            fclose(fid);
            lines = txt{1};
            for k = 1:length(lines)
                line = strtrim(lines{k});
                tokens = regexp(line, 'ORB_DECLARE\(([^)]+)\)', 'tokens', 'once');
                if ~isempty(tokens)
                    orbId = strtrim(tokens{1});
                    % store in cache and persist
                    obj.OrbCache.(snakeName) = orbId;
                    obj.saveOrbCache();
                    return;
                end
            end
        end

        function loadOrbCache(obj)
            % Load ORB id cache from disk if present and still valid.
            obj.OrbCacheLoaded = true;
            cachePath = fullfile(obj.LocalGeneratedDir, obj.OrbCacheFile);
            if ~exist(cachePath, 'file')
                obj.OrbCache = struct();
                return;
            end

            % Invalidate cache if any msg file is newer than cache file
            cacheInfo = dir(cachePath);
            cacheTime = cacheInfo.datenum;
            msgDir = fullfile(obj.PX4Root, 'msg');
            if exist(msgDir, 'dir')
                msgFiles = dir(fullfile(msgDir, '*.msg'));
                if ~isempty(msgFiles) && max([msgFiles(:).datenum]) > cacheTime
                    obj.OrbCache = struct();
                    return;
                end
            end

            try
                txt = fileread(cachePath);
                data = jsondecode(txt);
                % jsondecode returns struct or containers.Map-like; copy to OrbCache
                obj.OrbCache = data;
            catch
                obj.OrbCache = struct();
            end

            % If the cache looks incomplete compared to available message files,
            % try to rebuild it from the PX4 build headers which is much faster
            % than repeated per-topic find() calls.
            try
                msgDir = fullfile(obj.PX4Root, 'msg');
                msgFiles = dir(fullfile(msgDir, '*.msg'));
                numMsgs = length(msgFiles);
                cacheFields = fieldnames(obj.OrbCache);
                if numMsgs > 0 && length(cacheFields) < max(10, floor(0.8 * numMsgs))
                    obj.rebuildOrbCacheFromBuild();
                end
            catch
                % ignore rebuild failures
            end
        end

        function rebuildOrbCacheFromDisk(obj)
            % Scan PX4 source for uORB topic headers and extract ORB_DECLARE entries
            % This is much faster than repeated per-topic find() calls across 200+ topics
            obj.OrbCache = struct();
            
            % First priority: PX4 source code uORB topics (src/modules/uORB/topics/)
            srcTopicsDir = fullfile(obj.PX4Root, 'src', 'modules', 'uORB', 'topics');
            if exist(srcTopicsDir, 'dir')
                hdrs = dir(fullfile(srcTopicsDir, '*.h'));
                for i = 1:length(hdrs)
                    try
                        filePath = fullfile(hdrs(i).folder, hdrs(i).name);
                        txt = fileread(filePath);
                        tokens = regexp(txt, 'ORB_DECLARE\(([^)]+)\)', 'tokens');
                        if ~isempty(tokens)
                            orbId = strtrim(tokens{1}{1});
                            [~, topicName] = fileparts(hdrs(i).name);
                            obj.OrbCache.(topicName) = orbId;
                        end
                    catch
                        % skip files that can't be read
                    end
                end
            end
            
            % Second priority: PX4 build artifacts (if available)
            buildTopicsDir = fullfile(obj.PX4Root, 'build', 'px4_sitl_default', 'uORB', 'topics');
            if exist(buildTopicsDir, 'dir')
                hdrs = dir(fullfile(buildTopicsDir, '*.h'));
                for i = 1:length(hdrs)
                    try
                        filePath = fullfile(hdrs(i).folder, hdrs(i).name);
                        txt = fileread(filePath);
                        tokens = regexp(txt, 'ORB_DECLARE\(([^)]+)\)', 'tokens');
                        if ~isempty(tokens)
                            orbId = strtrim(tokens{1}{1});
                            [~, topicName] = fileparts(hdrs(i).name);
                            % Only update if not already found in source
                            if ~isfield(obj.OrbCache, topicName)
                                obj.OrbCache.(topicName) = orbId;
                            end
                        end
                    catch
                        % skip files that can't be read
                    end
                end
            end
            
            % For any messages without ORB_ID mapping, use the topic name as fallback
            % (This allows generation to proceed even if some topics can't be found)
            msgDir = fullfile(obj.PX4Root, 'msg');
            if exist(msgDir, 'dir')
                msgFiles = dir(fullfile(msgDir, '*.msg'));
                for i = 1:length(msgFiles)
                    [~, camelName] = fileparts(msgFiles(i).name);
                    topicName = px4API.camelCaseToSnakeCase(camelName);
                    if ~isfield(obj.OrbCache, topicName)
                        % Use topic name as fallback (will resolve to ORB_ID_<TOPIC_NAME>)
                        obj.OrbCache.(topicName) = upper(regexprep(topicName, '_', '_'));
                    end
                end
            end
            
            % Persist cache to disk
            obj.saveOrbCache();
            fprintf('✓ ORB cache rebuilt with %d entries\\n', length(fieldnames(obj.OrbCache)));
        end

        function rebuildOrbCacheFromBuild(obj)
            % Legacy method: scan PX4 build uORB topic headers and extract ORB_DECLARE entries
            % Use rebuildOrbCacheFromDisk() instead for better performance
            obj.OrbCache = struct();
            % search for build folders inside PX4 root
            findCmd = sprintf('find %s -path "*/build/*/uORB/topics/*.h" -print 2>/dev/null', obj.PX4Root);
            [status, out] = system(findCmd);
            if status ~= 0 || isempty(strtrim(out))
                % try default build path
                defaultPath = fullfile(obj.PX4Root, 'build');
                if exist(defaultPath, 'dir')
                    hdrs = dir(fullfile(defaultPath, '**', 'uORB', 'topics', '*.h'));
                    paths = arrayfun(@(x) fullfile(hdrs(x).folder, hdrs(x).name), 1:numel(hdrs), 'UniformOutput', false);
                else
                    paths = {};
                end
            else
                files = strsplit(strtrim(out), '\n');
                paths = files;
            end

            for i = 1:length(paths)
                p = paths{i};
                if isempty(p) || ~exist(p, 'file'), continue; end
                try
                    txt = fileread(p);
                    tokens = regexp(txt, 'ORB_DECLARE\(([^)]+)\)', 'tokens');
                    if ~isempty(tokens)
                        % choose first declared identifier
                        orbId = strtrim(tokens{1}{1});
                        [~, base] = fileparts(p);
                        % map base filename (snake topic) to declared orb id
                        obj.OrbCache.(base) = orbId;
                    end
                catch
                    % ignore file read errors
                end
            end
            % persist cache
            obj.saveOrbCache();
        end

        function saveOrbCache(obj)
            % Ensure directory exists
            if ~exist(obj.LocalGeneratedDir, 'dir')
                mkdir(obj.LocalGeneratedDir);
            end
            cachePath = fullfile(obj.LocalGeneratedDir, obj.OrbCacheFile);
            try
                fid = fopen(cachePath, 'w');
                if fid ~= -1
                    fprintf(fid, '%s', jsonencode(obj.OrbCache));
                    fclose(fid);
                end
            catch
                % ignore cache save errors
            end
        end

        function generateAllBussesAndHeaders(obj, outputDir)
            % Programmatically handles generation of workspace buses and strongly-typed C prototypes.
            % Uses a two-pass approach: collect all struct definitions, then output with forward declarations
            % to handle inter-struct dependencies.
            if nargin < 2 || isempty(outputDir)
                outputDir = obj.LocalGeneratedDir;
            end
            msgDir = fullfile(obj.PX4Root, 'msg');
            if ~exist(msgDir, 'dir')
                error('[px4API:Error] Could not find PX4 msg directory at: %s', msgDir);
            end

            msgFiles = dir(fullfile(msgDir, '*.msg'));
            fprintf('Found %d message profiles. Generating Simulink Buses & Structs...\n', length(msgFiles));

            % PASS 1: Collect all struct definitions
            allStructs = {};  % Will store {topicName, structStr, dependencies} triplets
            busAssignments = {};  % Will store Simulink bus assignments
            
            for i = 1:length(msgFiles)
                [~, camelName, ~] = fileparts(msgFiles(i).name);
                topicName = px4API.camelCaseToSnakeCase(camelName);  % Convert to snake_case

                try
                    [structString, busObj, deps] = obj.generateBusFromMsg(camelName, topicName);
                    if ~isempty(structString)
                        allStructs{end+1, 1} = topicName; %#ok<AGROW>
                        allStructs{end, 2} = structString; %#ok<AGROW>
                        allStructs{end, 3} = deps; %#ok<AGROW>
                        if ~isempty(busObj)
                            busAssignments{end+1, 1} = topicName; %#ok<AGROW>
                            busAssignments{end, 2} = busObj; %#ok<AGROW>
                        end
                    end
                catch ME
                    fprintf('! Skipping/Error in message [%s]: %s\n', topicName, ME.message);
                end
            end

            % PASS 2: Build header with forward declarations and definitions
            headerStr = sprintf('// Auto-generated by px4API for strongly-typed Simulink C Caller blocks\n');
            headerStr = sprintf('%s#ifndef PX4_SIMULINK_API_H\n#define PX4_SIMULINK_API_H\n\n', headerStr);
            headerStr = sprintf('%s#include <stdint.h>\n#include <stdbool.h>\n\n', headerStr);
            
            % Forward declare all structs to handle circular dependencies
            if ~isempty(allStructs)
                headerStr = sprintf('%s// Forward declarations to handle inter-struct dependencies\n', headerStr);
                for i = 1:size(allStructs, 1)
                    topicName = allStructs{i, 1};
                    headerStr = sprintf('%sstruct %s_s;\n', headerStr, topicName);
                end
                headerStr = sprintf('%s\n', headerStr);
            end
            
            % --- TYPE DEFINITION STRATEGY ---
            % For PX4 builds: NO struct definitions (they're in Test_types.h from Simulink)
            %                Use forward declarations and real uORB includes for glue code
            % For local simulation: use our generated struct definitions
            headerStr = sprintf('%s\n#if defined(__PX4_LINUX) || defined(__PX4_POSIX) || defined(__PX4_NUTTX)\n', headerStr);
            headerStr = sprintf('%s// PX4 Build: Skip struct definitions (Test_types.h provides them)\n', headerStr);
            headerStr = sprintf('%s// Forward declarations only for PX4\n', headerStr);
            % Note: struct definitions are in #else branch below
            
            headerStr = sprintf('%s#else\n', headerStr);
            headerStr = sprintf('%s// Local Simulation: use generated struct definitions\n\n', headerStr);

            % Output all struct definitions (now forward declarations exist)
            % First, sort structs by dependencies to ensure definitions come before usage
            if ~isempty(allStructs)
                allStructs = px4API.topologicalSortStructs(allStructs);
            end
            for i = 1:size(allStructs, 1)
                headerStr = sprintf('%s%s\n', headerStr, allStructs{i, 2});
            end
            
            headerStr = sprintf('%s\n#endif  // End PX4 vs Local struct definitions\n\n', headerStr);

            % Start appending clean, strongly-typed function signatures underneath with C Linkage
            headerStr = sprintf('%s#ifdef __cplusplus\nextern "C" {\n#endif\n\n', headerStr);
            headerStr = sprintf('%s// --- STRONGLY-TYPED RETURN-BY-VALUE PROTOTYPES FOR C CALLER ---\n', headerStr);
            
            srcStr = sprintf('// Empty stubs for Simulink simulation target parsing\n');
            srcStr = sprintf('#include "%s"\n\n', obj.StubHeaderName);

            % Append zero-input reader functions and single-input writer functions for all topics
            % Convert CamelCase filenames to snake_case immediately for consistent naming
            for i = 1:length(msgFiles)
                [~, camelName, ~] = fileparts(msgFiles(i).name);
                topicName = px4API.camelCaseToSnakeCase(camelName);  % Convert to snake_case
                
                % 1. Reader Prototype & Mock Source: Returns full structure layout by value
                headerStr = sprintf('%sstruct %s_s read_%s(void);\n', headerStr, topicName, topicName);
                srcStr = sprintf('%sextern "C" struct %s_s read_%s(void) { struct %s_s empty = {0}; return empty; }\n', srcStr, topicName, topicName, topicName);
                
                % 2. Writer Prototype & Mock Source: Accepts flat structure layout copy by value
                headerStr = sprintf('%svoid write_%s(struct %s_s in_buffer);\n', headerStr, topicName, topicName);
                srcStr = sprintf('%sextern "C" void write_%s(struct %s_s in_buffer) {}\n', srcStr, topicName, topicName);
            end

            headerStr = sprintf('%s\n#ifdef __cplusplus\n}\n#endif\n\n#endif // PX4_SIMULINK_API_H\n', headerStr);

            % 1. Write out the single combined stub header file locally
            if ~exist(outputDir, 'dir')
                mkdir(outputDir);
            end
            headerFilePath = fullfile(outputDir, obj.StubHeaderName);
            fid = fopen(headerFilePath, 'w');
            if fid == -1
                error('[px4API:Error] Could not write header file: %s', headerFilePath);
            end
            fprintf(fid, '%s', headerStr);
            fclose(fid);
            fprintf('✓ Successfully synchronized concrete header: %s\n', obj.StubHeaderName);

            % 2. Write out the matching strongly-typed source stubs file locally
            srcFilePath = fullfile(outputDir, 'px4_simulink_api.cpp');
            fid = fopen(srcFilePath, 'w');
            if fid == -1
                error('[px4API:Error] Could not write source file: %s', srcFilePath);
            end
            fprintf(fid, '%s', srcStr);
            fclose(fid);
            fprintf('✓ Successfully generated source stubs for C Caller: px4_simulink_api.cpp\n');

            % 3. Assign all Simulink bus objects to the workspace
            % (Now that forward declarations exist, all inter-struct references are valid)
            for i = 1:size(busAssignments, 1)
                topicName = busAssignments{i, 1};
                busObj = busAssignments{i, 2};
                nativeStructName = [topicName, '_s'];
                assignin('base', nativeStructName, busObj);
            end

            fprintf('--- [px4API] System Ready for Simulink Modelling ---\n\n');
        end


        function [structStr, busObj, dependencies] = generateBusFromMsg(obj, camelName, topicName)
            % Generate Simulink bus structure from PX4 .msg file
            % Returns: structStr (C struct definition), busObj (Simulink bus), dependencies (list of required structs)
            % camelName: original CamelCase filename (used to open the file)
            % topicName: snake_case topic name (used for struct naming)
            % 
            % If called with single argument (legacy), assume input is already camelName
            if nargin == 2
                topicName = px4API.camelCaseToSnakeCase(camelName);
            end
            
            structStr = '';
            busObj = [];
            dependencies = {};
            
            % Open file using the original CamelCase filename
            msgFilePath = fullfile(obj.PX4Root, 'msg', [camelName, '.msg']);
            fid = fopen(msgFilePath, 'r');
            if fid == -1
                error('[px4API:Error] Could not open message file: %s', msgFilePath);
            end
            fileData = textscan(fid, '%s', 'Delimiter', '\n');
            fclose(fid);
            lines = fileData{1};

            elements = [];
            structBody = sprintf('struct %s_s {\n', topicName);

            for i = 1:length(lines)
                line = strtrim(lines{i});
                if isempty(line) || startsWith(line, '#')
                    continue;
                end

                commentIdx = strfind(line, '#');
                if ~isempty(commentIdx)
                    line = strtrim(line(1:commentIdx(1)-1)); 
                end

                % Filter out static constant lines
                if contains(line, '=')
                    continue; 
                end

                tokens = strsplit(line);
                if length(tokens) < 2, continue; end

                px4Type = tokens{1};
                varName = tokens{2};

                % Clean up any trailing formatting artifacts
                if endsWith(varName, ';'), varName = varName(1:end-1); end

                % --- ROBUST ARRAY EXTRACTION MECHANISM ---
                arraySize = 1;

                % Extract array size from either type or name (e.g., "float32[4]" or "q[4]")
                arrayMatch = regexp([px4Type, ' ', varName], '\[(\d+)\]', 'tokens');
                if ~isempty(arrayMatch)
                    arraySize = str2double(arrayMatch{1}{1});
                end
                
                % Strip array notation from type and name
                px4Type = regexprep(px4Type, '\[\d+\]', '');
                varName = regexprep(varName, '\[\d+\]', '');

                % Safety Filter: Skip layout padding fields if encountered
                if startsWith(varName, 'sl_padding')
                    continue;
                end

                % Detect dependencies: Check if this type references another message struct
                % PX4 message types follow the pattern: TypeName or other known types
                [cType, slType, isDependency, depName] = obj.px4TypeToCTypesWithDeps(px4Type);
                if isDependency
                    % Record this dependency (convert CamelCase to snake_case)
                    depNameSnake = px4API.camelCaseToSnakeCase(depName);
                    if ~any(strcmp(dependencies, depNameSnake))
                        dependencies{end+1} = depNameSnake; %#ok<AGROW>
                    end
                end

                if arraySize > 1
                    structBody = sprintf('%s    %s %s[%d];\n', structBody, cType, varName, arraySize);
                else
                    structBody = sprintf('%s    %s %s;\n', structBody, cType, varName);
                end

                elem = Simulink.BusElement;
                elem.Name = varName;
                elem.DataType = slType;
                elem.Dimensions = arraySize;
                elem.Complexity = 'real';
                elements = [elements; elem]; %#ok<AGROW>
            end

            if ~isempty(elements)
                % Preserve the exact lowercase snake_case name of the message file
                % e.g., topicName = 'vehicle_local_position' -> 'vehicle_local_position_s'
                nativeStructName = [lower(topicName), '_s'];
                
                structBody = sprintf('%s};\ntypedef struct %s %s;\n', structBody, nativeStructName, nativeStructName);
                structStr = structBody;

                % Create bus object (will be assigned to workspace later after all structs defined)
                busObj = Simulink.Bus;
                busObj.Elements = elements;
            end
        end

        function exportGeneratedCode(obj, buildInfo)
            fprintf('\n--- [px4API] Starting Automated Clean & Export ---\n');
            fprintf('Local Generated Folder: %s\n', obj.LocalGeneratedDir);
            fprintf('Target PX4 Module  : %s\n', obj.ResolvedExternalDir);

            % If no extensions configured, do not proceed with export.
            if isempty(obj.AllowedExtensions)
                fprintf('! AllowedExtensions is empty — nothing to export. Aborting.\n');
                return;
            end

            if ~exist(obj.PX4Root, 'dir')
                error('[px4API:Error] The specified PX4 root directory does not exist: %s', obj.PX4Root);
            end

            % HOUSEKEEPING: Force-purge old folder to completely erase legacy artifacts
            if exist(obj.ResolvedExternalDir, 'dir')
                fprintf('Purging legacy generated folder tree...\n');
                rmdir(obj.ResolvedExternalDir, 's');
            end
            mkdir(obj.ResolvedExternalDir);

            % Enforce presence of the secondary local asset directory if utilized
            if ~exist(obj.LocalGeneratedDir, 'dir')
                fprintf('! Local generated folder missing, preparing artifacts first...\n');
                obj.prepareLocalGeneratedArtifacts();
            end

            % Extract model component name natively from buildInfo
            modelName = buildInfo.ComponentName;
            if isempty(modelName)
                error('[px4API:Error] buildInfo must provide ComponentName.');
            end

            % Generate the model-specific glue locally from the live Simulink model.
            obj.generateOmnipotentCppGlue(obj.LocalGeneratedDir);
            filesExportedCount = 0;
            filesExportedCount = filesExportedCount + obj.copyGeneratedCodeFiles(obj.LocalGeneratedDir, obj.ResolvedExternalDir, false);

            buildDir = fullfile(obj.MatlabProjectRoot, sprintf("%s_ert_rtw", modelName));

            % Copy the files Simulink generated for this model into PX4.
            filesExportedCount = filesExportedCount + obj.copyGeneratedCodeFiles(buildDir, obj.ResolvedExternalDir, false);

            fprintf('Successfully moved %d generated files over to PX4 code tree.\n', filesExportedCount);
        end

        function count = copyGeneratedCodeFiles(obj, sourceDir, destDir, recursive)
            % Copy generated artifacts from sourceDir into destDir.
            % If recursive is true, recurse into subdirectories; otherwise only copy files in sourceDir.
            if nargin < 4
                recursive = false;
            end
            if ~exist(sourceDir, 'dir')
                count = 0;
                return;
            end
            if ~exist(destDir, 'dir')
                mkdir(destDir);
            end

            count = 0;
            entries = dir(sourceDir);
            extAllowed = cellfun(@lower, obj.AllowedExtensions, 'UniformOutput', false);
            
            for i = 1:length(entries)
                if entries(i).name(1) == '.'
                    continue;  % skip . and ..
                end

                sourcePath = fullfile(entries(i).folder, entries(i).name);
                destPath = fullfile(destDir, entries(i).name);

                if entries(i).isdir
                    if recursive
                        count = count + obj.copyGeneratedCodeFiles(sourcePath, destPath, true);
                    end
                else
                    % Only copy files matching AllowedExtensions property
                    [~, ~, ext] = fileparts(entries(i).name);
                    if ismember(lower(ext), extAllowed)
                        copyfile(sourcePath, destPath, 'f');
                        count = count + 1;
                    end
                end
            end
        end

        function generateOmnipotentCppGlue(obj, outputDir)
            % Scans Test.cpp to find which read_*/write_* functions are actually used,
            % then generates glue code ONLY for those topics (not all 201).
            if nargin < 2 || isempty(outputDir)
                outputDir = obj.ResolvedExternalDir;
            end
            fprintf('\n--- [px4API] Generating Selective C++ uORB Glue Code (functions used only) ---\n');

            % Step 1: Scan generated model code to find which functions are called
            requiredTopics = {};
            testCppPath = fullfile(obj.MatlabProjectRoot, 'Test_ert_rtw', 'Test.cpp');
            if isfile(testCppPath)
                fid = fopen(testCppPath, 'r');
                testContent = fread(fid, '*char')';
                fclose(fid);
                
                % Find all read_TOPIC() and write_TOPIC() calls
                readMatches = regexp(testContent, 'read_(\w+)\s*\(', 'tokens');
                writeMatches = regexp(testContent, 'write_(\w+)\s*\(', 'tokens');
                
                % Flatten cell array and unique-ify
                allMatches = [readMatches; writeMatches];
                if ~isempty(allMatches)
                    requiredTopics = unique([allMatches{:}]);
                end
                
                fprintf('  Found %d unique topics used in model\n', length(requiredTopics));
            else
                fprintf('  Warning: Could not find Test.cpp, generating for all topics\n');
                msgDir = fullfile(obj.PX4Root, 'msg');
                msgFiles = dir(fullfile(msgDir, '*.msg'));
                for i = 1:length(msgFiles)
                    [~, camelName, ~] = fileparts(msgFiles(i).name);
                    requiredTopics{i} = px4API.camelCaseToSnakeCase(camelName);
                end
            end

            % Start building the C++ source file
            cppStr = sprintf('// Auto-generated selective strongly-typed return-by-value uORB routing layer\n');
            cppStr = sprintf('%s#include <px4_platform_common/log.h>\n#include <uORB/uORB.h>\n', cppStr);
            cppStr = sprintf('%s#include <string.h>\n', cppStr);
            cppStr = sprintf('%s#include "px4_simulink_api.h"\n', cppStr);
            
            % Include real uORB topic headers for glue function implementations
            for i = 1:length(requiredTopics)
                topicName = requiredTopics{i};
                cppStr = sprintf('%s#include <uORB/topics/%s.h>\n', cppStr, topicName);
            end

            % --- THE LINUX LINKER FIXED PASS ---
            % Enforce pure C linkage output rules for ALL generated implementation blocks.
            % This prevents the C++ compiler from mangling function signatures, which resolves 
            % the "undefined reference" errors during the final bin/px4 link step.
            cppStr = sprintf('%s\nextern "C" {\n\n', cppStr);

            % =========================================================================
            % 1. RETURN-BY-VALUE READER FUNCTIONS (for required topics only)
            % =========================================================================
            % Returning the structure directly by value forces the Simulink C Caller to 
            % recognize the function as a pure output node, removing the dual out_buffer ports.
            for i = 1:length(requiredTopics)
                topicName = requiredTopics{i};
                orbId = obj.findOrbIdForTopic(topicName);
                
                cppStr = sprintf('%sstruct %s_s read_%s(void) {\n', cppStr, topicName, topicName);
                cppStr = sprintf('%s    static int sub_handle = -1;\n', cppStr);
                cppStr = sprintf('%s    if (sub_handle < 0) { sub_handle = orb_subscribe(ORB_ID(%s)); }\n', cppStr, orbId);
                cppStr = sprintf('%s    static struct %s_s local_buffer;\n', cppStr, topicName);
                cppStr = sprintf('%s    bool updated = false;\n', cppStr);
                cppStr = sprintf('%s    orb_check(sub_handle, &updated);\n', cppStr);
                cppStr = sprintf('%s    if (updated) { orb_copy(ORB_ID(%s), sub_handle, &local_buffer); }\n', cppStr, orbId);
                cppStr = sprintf('%s    return local_buffer;\n}\n\n', cppStr);
            end

            % =========================================================================
            % 2. PASS-BY-VALUE WRITER FUNCTIONS (for required topics only)
            % =========================================================================
            % Accepting the struct copy directly by value natively places a single input port
            % arrow on the left-hand face of the C Caller block without triggering parameter scope leaks.
            for i = 1:length(requiredTopics)
                topicName = requiredTopics{i};
                orbId = obj.findOrbIdForTopic(topicName);
                
                cppStr = sprintf('%svoid write_%s(struct %s_s in_buffer) {\n', cppStr, topicName, topicName);
                cppStr = sprintf('%s    static orb_advert_t pub_handle = nullptr;\n', cppStr);
                cppStr = sprintf('%s    if (pub_handle == nullptr) { pub_handle = orb_advertise(ORB_ID(%s), &in_buffer); }\n', cppStr, orbId);
                cppStr = sprintf('%s    else { orb_publish(ORB_ID(%s), pub_handle, &in_buffer); }\n', cppStr, orbId);
                cppStr = sprintf('%s}\n\n', cppStr);
            end
            
            % Close the C Linkage macro bracket block safely
            cppStr = sprintf('%s}\n', cppStr);

            % Deploy the omnipotent glue to the target output directory layout path
            if ~exist(outputDir, 'dir')
                mkdir(outputDir);
            end
            glueFilePath = fullfile(outputDir, 'simulink_io_glue.cpp');
            fid = fopen(glueFilePath, 'w');
            if fid == -1
                error('[px4API:Error] Could not write glue file: %s', glueFilePath);
            end
            fprintf(fid, '%s', cppStr);
            fclose(fid);
            fprintf('✓ Successfully wrote omnipotent uORB router: %s\n\n', glueFilePath);
        end


    end

    methods (Static)
        % ===== Helper to convert CamelCase .msg filenames to snake_case uORB topics =====
        % PX4 firmware v1.16+ uses CamelCase for .msg filenames (e.g., SensorAirflow.msg)
        % but internally uORB topics are snake_case (e.g., sensor_airflow)
        function snakeName = camelCaseToSnakeCase(camelName)
            % Convert CamelCase to snake_case: insert underscore before capital letters,
            % then convert to lowercase
            % Example: SensorAirflow -> sensor_airflow
            snakeName = regexprep(camelName, '([a-z])([A-Z])', '$1_$2');
            snakeName = lower(snakeName);
        end

        function [cType, slType] = px4TypeToCTypes(px4Type)
            % Map PX4 msg primitive types to C and Simulink types.
            switch px4Type
                case 'float32', slType = 'single';   cType = 'float';
                case 'float64', slType = 'double';   cType = 'double';
                case 'uint64',  slType = 'uint64';   cType = 'uint64_t';
                case 'uint32',  slType = 'uint32';   cType = 'uint32_t';
                case 'uint16',  slType = 'uint16';   cType = 'uint16_t';
                case 'uint8',   slType = 'uint8';    cType = 'uint8_t';
                case 'int64',   slType = 'int64';    cType = 'int64_t';
                case 'int32',   slType = 'int32';    cType = 'int32_t';
                case 'int16',   slType = 'int16';    cType = 'int16_t';
                case 'int8',    slType = 'int8';     cType = 'int8_t';
                case 'bool',    slType = 'boolean';  cType = 'bool';
                case 'char',    slType = 'char';     cType = 'char';
                otherwise
                    % Non-primitive: assume an embedded message type.
                    % Use struct <Type>_s as C type and <Type>_s as Simulink bus name.
                    % Preserve original casing for struct name; Simulink bus uses same name.
                    slType = [px4Type, '_s'];
                    cType = ['struct ', px4Type, '_s'];
            end
        end

        function [cType, slType, isDependency, depName] = px4TypeToCTypesWithDeps(px4Type)
            % Map PX4 msg primitive types to C/Simulink types and detect message dependencies
            % isDependency: true if this references another message struct
            % depName: the CamelCase name of the referenced message struct (if isDependency=true)
            isDependency = false;
            depName = '';
            
            switch px4Type
                case 'float32', slType = 'single';   cType = 'float';
                case 'float64', slType = 'double';   cType = 'double';
                case 'uint64',  slType = 'uint64';   cType = 'uint64_t';
                case 'uint32',  slType = 'uint32';   cType = 'uint32_t';
                case 'uint16',  slType = 'uint16';   cType = 'uint16_t';
                case 'uint8',   slType = 'uint8';    cType = 'uint8_t';
                case 'int64',   slType = 'int64';    cType = 'int64_t';
                case 'int32',   slType = 'int32';    cType = 'int32_t';
                case 'int16',   slType = 'int16';    cType = 'int16_t';
                case 'int8',    slType = 'int8';     cType = 'int8_t';
                case 'bool',    slType = 'boolean';  cType = 'bool';
                case 'char',    slType = 'char';     cType = 'char';
                otherwise
                    % Non-primitive: assume an embedded message type (struct reference)
                    isDependency = true;
                    depName = px4Type;  % Store original CamelCase name for dependency tracking
                    % Convert struct name to snake_case for actual C type
                    structNameSnake = px4API.camelCaseToSnakeCase(px4Type);
                    slType = [structNameSnake, '_s'];
                    cType = ['struct ', structNameSnake, '_s'];
            end
        end

        function orderedStructs = topologicalSortStructs(allStructs)
            % Topologically sort structs by dependencies
            % Input: allStructs {topicName, structStr, dependencies}
            % Output: orderedStructs {topicName, structStr} sorted so dependencies come first
            
            if isempty(allStructs)
                orderedStructs = {};
                return;
            end
            
            numStructs = size(allStructs, 1);
            struct_map = containers.Map();  % topicName -> index
            for i = 1:numStructs
                struct_map(allStructs{i, 1}) = i;
            end
            
            % Track visited nodes
            visited = false(numStructs, 1);
            orderedStructs = {};
            
            % Depth-first traversal to build dependency order
            for i = 1:numStructs
                if ~visited(i)
                    [orderedStructs, visited] = px4API.dfs_visit(i, allStructs, struct_map, visited, orderedStructs);
                end
            end
        end
    end
    
    methods (Static)
        function [orderedStructs, visited] = dfs_visit(idx, allStructs, struct_map, visited, orderedStructs)
            % DFS helper for topological sort
            % Returns updated orderedStructs and visited arrays
            if visited(idx)
                return;
            end
            
            visited(idx) = true;
            topicName = allStructs{idx, 1};
            dependencies = allStructs{idx, 3};
            
            % Visit dependencies first
            for i = 1:length(dependencies)
                depName = dependencies{i};
                if struct_map.isKey(depName)
                    depIdx = struct_map(depName);
                    if ~visited(depIdx)
                        [orderedStructs, visited] = px4API.dfs_visit(depIdx, allStructs, struct_map, visited, orderedStructs);
                    end
                end
            end
            
            % Add current struct after its dependencies
            orderedStructs{end+1, 1} = topicName; %#ok<AGROW>
            orderedStructs{end, 2} = allStructs{idx, 2}; %#ok<AGROW>
        end
    end
    
    methods (Static)
        function listStr = getTopicDropdownString()
            % Fast, static dropdown popup content manager
            % Returns comma-separated list of all available PX4 message topics (snake_case)
            api = px4API();
            msgDir = fullfile(api.PX4Root, 'msg');
            if ~exist(msgDir, 'dir')
                listStr = 'vehicle_local_position,sensor_combined';
                return;
            end

            files = dir(fullfile(msgDir, '*.msg'));
            topics = {};
            for i = 1:length(files)
                [~, camelName, ~] = fileparts(files(i).name);
                % Convert CamelCase filename to snake_case uORB topic name
                topicName = px4API.camelCaseToSnakeCase(camelName);
                topics{end+1} = topicName; %#ok<AGROW>
            end

            topics = unique(topics(~cellfun(@isempty, topics)));
            listStr = strjoin(topics, ',');
        end

        function runPostCodeGen(buildInfo, ~)
            % Hook called instantly upon Simulink code-gen complete
            apiInstance = px4API();
            apiInstance.exportGeneratedCode(buildInfo);
        end
        
        function uorb_topic_callback(callbackContext)
            % FIXED: Calls static method via class name and splits string into a cell array safely
            blockHandle = callbackContext.BlockHandle;
            choices = strsplit(px4API.getTopicDropdownString(), ',');
            set_param(blockHandle, 'TypeOptions_uorb_topic', choices);
        end
    end
end
