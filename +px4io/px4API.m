% px4API - PX4/Simulink Integration Code Generator
%
% This class generates strongly-typed C++ glue code and Simulink bus definitions
% for seamless PX4 uORB topic communication and parameter access from Simulink models.
%
% Key responsibilities:
%   1. Parse PX4 .msg files and generate Simulink bus definitions
%   2. Build C++ wrapper functions (read_*/write_*) for uORB topics
%   3. Generate parameter access functions (read_px4_param_*, write_px4_param_*)
%   4. Manage ORB topic ID cache for efficient code generation
%
% Usage:
%   api = px4API();          % Initialize (auto-regenerates if needed)

classdef px4API < handle
    properties
        % ===== USER CONFIGURATION =====
        % Absolute path to PX4 firmware repository root
        PX4Root = fullfile('~', 'PX4', 'v1.17.0-mod')

        % Name of target PX4 module (directory in src/modules/)
        PX4ModuleName = 'simulink_io'

        % Supported file extensions for export
        AllowedExtensions = {'.cpp', '.h'}        

        % Enable/disable debug output
        % Set to false to suppress initialization messages and progress output
        % Useful when calling px4API() from mask initialization (silent operation)
        ShowDebug = false
    end

    properties (Access = private)
        % ===== INTERNAL CACHE & STATE =====
        % Absolute path to MATLAB project directory containing this file
        PackageRoot = ''

        % Resolved path to PX4 generated code directory (PX4Root/src/modules/simulink_io/generated_code)
        ResolvedExternalDir = ''
        
        % Local directory for generated artifacts
        LocalGeneratedDir = ''
        EnumsDir = '';

        % Name of the generated stub header file
        StubHeaderName = 'px4_simulink_api.h'

        % In-memory cache of uORB topic metadata (JSON-serializable struct)
        OrbCache = struct()

        % Flag indicating whether cache has been loaded from disk
        OrbCacheLoaded = false

        % Filename for persistent ORB cache (stored in LocalGeneratedDir)
        OrbCacheFile = 'orb_id_cache.json'
    end
    methods
        % ========== PUBLIC METHODS ==========

        function obj = px4API()
            % Constructor - Initializes px4API and prepares code generation environment.
            %
            % This constructor:
            %   1. Resolves all paths to absolute locations
            %   2. Ensures LocalGeneratedDir exists
            %   3. Regenerates Simulink bus objects in workspace
            %   4. Auto-regenerates generated code if sources are newer than artifacts
            %
            % The generation is incremental - skipped if generated files are newer than
            % both PX4 messages and generator sources (px4API.m, uORB_*.m).

            % Fetch the full, absolute file system path of this specific script file
            %    Returns something like: '/path/to/project_root/+px4io/px4API'
            currentFilePath = mfilename('fullpath');

            % Step out of the 'px4API' filename context to find the '+px4io' folder lane
            [obj.PackageRoot, ~, ~] = fileparts(currentFilePath);

            % Enforce absolute resolution for the user-supplied PX4 root folder
            obj.PX4Root = obj.resolveAbsolutePath(obj.PX4Root);

            % Compute the exact target folder using native PX4 internal layout
            obj.ResolvedExternalDir = fullfile(obj.PX4Root, 'src', 'modules', ...
                                               obj.PX4ModuleName, 'generated_code');

            if obj.ShowDebug
                fprintf('\n--- [px4API] Initializing & Scanning PX4 Message Directory ---\n');
            end            

            % Only regenerate files if needed (timestamp-based check)
            [needsGeneration, generationReason] = obj.needsGeneration();
            if needsGeneration
                if obj.ShowDebug
                    fprintf('! Regenerating generated artifacts: %s\n', generationReason);
                end
                obj.generateAllBussesAndHeaders();
            else
                if obj.ShowDebug
                    fprintf('✓ Generated artifacts present and up-to-date; skipping file regeneration. (%s)\n', generationReason);
                end
                % Load cache from persistent JSON file if it exists
                obj.loadOrbCacheFromJson();

                % regenerate bus objects in workspace (they don't persist across clears)
                obj.regenerateBusesInWorkspace();
            end
        end

         

        function regenerateBusesInWorkspace(obj)
            % Regenerate Simulink bus objects and assign to MATLAB base workspace.
            %
            % Important: This is ALWAYS called (even on cache hits) because bus objects
            % are in-memory MATLAB objects and don't persist across workspace clears.
            % Files are NOT regenerated here - only in-memory objects are created.
            %
            % Buses are named <topic_name>_s to match C struct names.
            %
            % Errors during bus generation are silently skipped to allow partial
            % regeneration (e.g., if some topics have syntax issues).

            % Try Loading from json first since it is much faster
            if obj.OrbCacheLoaded && isfield(obj.OrbCache, 'topics') && ~isempty(fieldnames(obj.OrbCache.topics))
                if obj.ShowDebug
                    fprintf('Loading Simulink bus types directly from active OrbCache memory structure...\n');
                end
                topicsList = fieldnames(obj.OrbCache.topics);
                busCount = 0;
                
                for idx = 1:length(topicsList)
                    tName = topicsList{idx};
                    nativeStructName = [tName, '_s'];
                    topicData = obj.OrbCache.topics.(tName);
                    
                    % Verify that this specific topic data structure contains a 'fields' sub-array
                    if isfield(topicData, 'fields') && ~isempty(topicData.fields)
                        fieldsData = topicData.fields;
                        
                        % Reuse the shared helper to build the bus object directly
                        busObj = obj.createBusFromFieldData(fieldsData);
                        if ~isempty(busObj)
                            assignin('base', nativeStructName, busObj);
                            busCount = busCount + 1;
                        end
                    end
                end
                
                if obj.ShowDebug
                    fprintf('✓ Restored %d Simulink bus types from existing memory cache keys\n', busCount);
                end
            else
                if obj.ShowDebug
                    fprintf('x No Simulink buses were loaded from cache\n');
                end
            end
        end

        function [needed, reason] = needsGeneration(obj)
            % Determine whether code regeneration is necessary (timestamp-based check).
            %
            % Validates folder layouts. If any generated files are missing, it safely
            % clears the contents of the target directories without deleting the folders 
            % themselves, avoiding MATLAB path corruption warnings.

            needed = true;

            % 1. DEFINE PATH TARGET CONTEXTS RELATIVE TO PACKAGE
            obj.LocalGeneratedDir = fullfile(obj.PackageRoot, 'generated_code');
            obj.EnumsDir = fullfile(obj.PackageRoot, '+enums');

            % 2. ENFORCE NATIVE SELF-HEALING FOLDER STRUCTURE
            % Ensure physical directories exist. We DO NOT add "+enums" to the MATLAB path.
            if ~exist(obj.LocalGeneratedDir, 'dir') || ~exist(obj.EnumsDir, 'dir')
                reason = 'mandatory package directories are missing';
                
                % Create missing folder spaces safely without deleting existing ones
                if ~exist(obj.LocalGeneratedDir, 'dir'), mkdir(obj.LocalGeneratedDir); end
                if ~exist(obj.EnumsDir, 'dir'), mkdir(obj.EnumsDir); end
                
                % Only add the local non-package generated directory to the path if missing
                if isempty(strfind(path(), obj.LocalGeneratedDir))
                    addpath(obj.LocalGeneratedDir);
                end
                return; % Exit early to trigger immediate clean file generation pass
            end

            % Ensure the regular generated code directory is on the path
            if isempty(strfind(path(), obj.LocalGeneratedDir))
                addpath(obj.LocalGeneratedDir);
            end

            % 3. RUN ARCHITECTURAL BASELINE FILE CHECKS
            hdrPath = fullfile(obj.LocalGeneratedDir, obj.StubHeaderName);
            srcPath = fullfile(obj.LocalGeneratedDir, 'px4_simulink_api.cpp');
            checkPath = fullfile(obj.LocalGeneratedDir, obj.OrbCacheFile);

            if ~(exist(hdrPath, 'file') == 2 && exist(srcPath, 'file') == 2 && exist(checkPath, 'file') == 2)
                reason = 'one or more generated source files are missing';
                
                % SAFE PURGE: Erase ONLY the files inside, leaving folder links completely locked
                % This prevents MATLAB path removal warnings.
                obj.clearFolderContents(obj.LocalGeneratedDir);
                obj.clearFolderContents(obj.EnumsDir);
                return;
            end

            msgDir = fullfile(obj.PX4Root, 'msg');
            if ~exist(msgDir, 'dir')
                reason = 'PX4 msg directory is missing';
                return;
            end

            % 4. CAPTURE MODIFICATION TIMESTAMPS
            msgFiles = dir(fullfile(msgDir, '**', '*.msg'));
            if isempty(msgFiles)
                needed = false;
                reason = 'no PX4 .msg files were found';
                return;
            end
            
            % Exclude px4_msgs_old folder trees from timestamp checks
            excludePattern = [filesep 'px4_msgs_old']; 
            isOldMessage = contains({msgFiles.folder}, excludePattern);
            msgFiles(isOldMessage) = [];
            
            newestMsg = max([msgFiles(:).datenum]);

            hdrInfo = dir(hdrPath);
            srcInfo = dir(srcPath);
            glueInfo = dir(checkPath);
            hdrTime = hdrInfo.datenum;
            srcTime = srcInfo.datenum;
            glueTime = glueInfo.datenum;

            generatorTimes = [];
            generatorFiles = {
                fullfile(obj.PackageRoot, 'px4API.m')
                fullfile(obj.PackageRoot, 'uORB_read.m')
                fullfile(obj.PackageRoot, 'uORB_write.m')
                fullfile(obj.PackageRoot, 'uORB_msg.m')
                fullfile(obj.PackageRoot, 'uORB_time.m')
            };
            for i = 1:numel(generatorFiles)
                filePath = generatorFiles{i};
                if exist(filePath, 'file') == 2
                    info = dir(filePath);
                    if ~isempty(info)
                        generatorTimes(end+1) = info(1).datenum; %#ok<AGROW>
                    end
                end
            end
            
            if isempty(generatorTimes)
                newestGeneratorTime = -inf;
            else
                newestGeneratorTime = max(generatorTimes);
            end

            % 5. DYNAMIC EVALUATION DECISION LOOP
            newestDependencyTime = max(newestMsg, newestGeneratorTime);
            if hdrTime >= newestDependencyTime && srcTime >= newestDependencyTime && glueTime >= newestDependencyTime
                needed = false;
                reason = 'generated files are newer than PX4 messages and generator sources';
            else
                needed = true;
                reason = 'PX4 messages or generator sources are newer than generated files';
                
                % Safe incremental pre-clear before regeneration
                obj.clearFolderContents(obj.LocalGeneratedDir);
                obj.clearFolderContents(obj.EnumsDir);
            end
        end

        function clearFolderContents(~, folderPath)
            % Sweeps a target directory and safely deletes only files and sub-items 
            % while keeping the parent directory handle intact to prevent path warnings.
            if exist(folderPath, 'dir') == 7
                items = dir(folderPath);
                for i = 1:length(items)
                    itemName = items(i).name;
                    % Skip current directory (.) and parent directory (..) references
                    if strcmp(itemName, '.') || strcmp(itemName, '..'), continue; end

                    fullItemPath = fullfile(items(i).folder, itemName);
                    if items(i).isdir
                        rmdir(fullItemPath, 's'); % Safely remove nested subdirs if any exist
                    else
                        delete(fullItemPath);     % Erase file entry directly
                    end
                end
            end
        end


        function fieldMetadata = getFieldMetadataFromCache(obj, topicName)
            % Retrieve field metadata for a topic from the persistent JSON cache.
            %
            % Returns a table with columns:
            %   - fieldName (string): C field name
            %   - fieldType (string): PX4 type (e.g., 'float32', 'int32')
            %   - arraySize (numeric): 1 for scalar, >1 for arrays
            %
            % Input:
            %   topicName - Topic name in snake_case
            %
            % Output:
            %   fieldMetadata - Table (empty if topic not in cache)
            %
            % Usage: Used by init_* and generate C code to set NaN on float fields

            fieldMetadata = table();

            if isfield(obj.OrbCache, 'topics') && isfield(obj.OrbCache.topics, topicName)
                topicEntry = obj.OrbCache.topics.(topicName);
                if isfield(topicEntry, 'fields')
                    fieldsArray = topicEntry.fields;
                    if iscell(fieldsArray)
                        fieldsArray = fieldsArray{1};  % Unwrap cell if needed
                    end
                    if isstruct(fieldsArray)
                        if isempty(fieldsArray)
                            return;
                        end
                        fieldNames = {fieldsArray(:).name};
                        fieldTypes = {fieldsArray(:).type};
                        arraySizes = [fieldsArray(:).arraySize];

                        fieldMetadata = table(fieldNames', fieldTypes', arraySizes', ...
                            'VariableNames', {'fieldName', 'fieldType', 'arraySize'});
                    end
                end
            end
        end

        function variants = getTopicVariants(obj, topicName)
            % Return the set of actual uORB topic IDs for a message base or variant.
            %
            % If no TOPICS metadata is present, the returned list is the requested topic name.
            variants = {topicName};
            if ~isfield(obj.OrbCache, 'topics')
                return;
            end

            if isfield(obj.OrbCache.topics, topicName)
                entry = obj.OrbCache.topics.(topicName);
                if isfield(entry, 'variants') && ~isempty(entry.variants)
                    variants = entry.variants;
                end
                return;
            end

            % If the input is itself a variant name, return all variants of its base message.
            topicKeys = fieldnames(obj.OrbCache.topics);
            for i = 1:length(topicKeys)
                baseName = topicKeys{i};
                entry = obj.OrbCache.topics.(baseName);
                if isfield(entry, 'variants') && any(strcmp(entry.variants, topicName))
                    variants = entry.variants;
                    return;
                end
            end
        end

        function baseTopic = getBaseTopicForVariant(obj, topicName)
            % Map a variant topic name back to its base message topic name.
            baseTopic = topicName;
            if ~isfield(obj.OrbCache, 'topics')
                return;
            end

            if isfield(obj.OrbCache.topics, topicName)
                return;
            end

            topicKeys = fieldnames(obj.OrbCache.topics);
            for i = 1:length(topicKeys)
                key = topicKeys{i};
                entry = obj.OrbCache.topics.(key);
                if isfield(entry, 'variants') && any(strcmp(entry.variants, topicName))
                    baseTopic = key;
                    return;
                end
            end
        end

        function saveOrbCache(obj)
            % Save ORB cache to disk as JSON.
            %
            % Format:
            %   {
            %     "timestamp": "ISO8601 timestamp",
            %     "topics": {
            %       "topic_name": {
            %         "fields": [
            %           {"name": "field1", "type": "float32", "arraySize": 1},
            %           ...
            %         ]
            %       },
            %       ...
            %     }
            %   }
            %
            % Errors during save are silently ignored.

            if ~exist(obj.LocalGeneratedDir, 'dir')
                mkdir(obj.LocalGeneratedDir);
            end
            cachePath = fullfile(obj.LocalGeneratedDir, obj.OrbCacheFile);
            % Ensure cache has timestamp and topics structure
            if ~isfield(obj.OrbCache, 'timestamp')
                obj.OrbCache.timestamp = datetime('now', 'Format', 'yyyy-MM-dd''T''HH:mm:ss''Z''');
            end
            if ~isfield(obj.OrbCache, 'topics')
                obj.OrbCache.topics = struct();
            end

            fid = fopen(cachePath, 'w');
            if fid ~= -1
                % Pretty-print JSON for human readability
                jsonStr = jsonencode(obj.OrbCache, 'PrettyPrint', true);
                fprintf(fid, '%s', jsonStr);
                fclose(fid);
            end
        end

        function loadOrbCacheFromJson(obj)
            % Load ORB cache from persistent JSON file if it exists.
            %
            % Called when generated files are up-to-date to restore the in-memory cache
            % without rebuilding from source messages. This preserves variant metadata
            % and field information.
            %
            % Errors during load are silently ignored; cache remains empty.

            cachePath = fullfile(obj.LocalGeneratedDir, obj.OrbCacheFile);
            if ~exist(cachePath, 'file')
                return;  % No cache file to load
            end

            jsonStr = fileread(cachePath);
            obj.OrbCache = jsondecode(jsonStr);
            % Ensure topics field exists even if JSON is empty/malformed
            if ~isfield(obj.OrbCache, 'topics')
                obj.OrbCache.topics = struct();
            end
            obj.OrbCacheLoaded = true;
        end

        function generateAllBussesAndHeaders(obj)
            % Generate Simulink bus definitions and C++ stub headers for all messages.
            %
            % This is the first major code generation pipeline stage.
            % Creates:
            %   1. px4_simulink_api.h - C prototypes for all topics (read/write/init functions)
            %   2. px4_simulink_api.cpp - C++ source stubs (for PX4 linking)
            %   3. ORB cache update - Field metadata for all topics (used by NaN init code)
            %
            % Process:
            %   - Pass 1: Parse all .msg files and collect struct definitions
            %   - Pass 2: Topologically sort structs by dependencies (no forward references)
            %   - Pass 3: Emit C++ code with proper declarations
            %   - Side effect: Simulink buses are regenerated in workspace (via regenerateBusesInWorkspace)
            %
            % Input:
            %   outputDir - Directory for generated .h/.cpp files (default: LocalGeneratedDir)
            msgDir = fullfile(obj.PX4Root, 'msg');
            if ~exist(msgDir, 'dir')
                error('[px4API:Error] Could not find PX4 msg directory at: %s', msgDir);
            end

            % Scans ALL subdirectories recursively to catch versioned or hidden messages.
            rawMsgFiles = dir(fullfile(msgDir, '**', '*.msg'));

            % Filter out files residing inside the 'px4_msgs_old' directory tree.
            % We look for both forward and backslashes to ensure it works on Linux/Ubuntu and Windows.
            excludePattern = [filesep 'px4_msgs_old']; 
            isOldMessage = contains({rawMsgFiles.folder}, excludePattern);
            rawMsgFiles(isOldMessage) = []; % Deletes matching stale entries instantly

            % Deduplicate by topic name to keep the first matching profile per topic.
            msgFiles = [];
            processedTopics = {};
            for idx = 1:length(rawMsgFiles)
                [~, camelName, ~] = fileparts(rawMsgFiles(idx).name);
                topicName = obj.camelCaseToSnakeCase(camelName);
                if ~any(strcmp(processedTopics, topicName))
                    processedTopics{end+1} = topicName; %#ok<AGROW>
                    msgFiles = [msgFiles; rawMsgFiles(idx)]; %#ok<AGROW>
                end
            end

            if obj.ShowDebug
                fprintf('Found %d total unique message profiles across all subfolders. Generating Simulink Buses...\n', length(msgFiles));
            end

            % PASS 1: Collect all struct definitions and build comprehensive metadata cache
            allStructs = {};  % Will store {topicName, structStr, dependencies} triplets
            busAssignments = {};  % Will store Simulink bus assignments

            % Initialize the comprehensive cache structure
            if ~isfield(obj.OrbCache, 'topics')
                obj.OrbCache.topics = struct();
            end

            for i = 1:length(msgFiles)
                msgFilePath = fullfile(msgFiles(i).folder, msgFiles(i).name);
                [~, camelName, ~] = fileparts(msgFiles(i).name);
                topicName = obj.camelCaseToSnakeCase(camelName);  % Convert to snake_case

                try
                    [structString, busObj, deps, fieldMeta, topicVariants] = obj.generateBusFromMsg(camelName, topicName, msgFilePath);
                    if ~isempty(structString)
                        allStructs{end+1, 1} = topicName; %#ok<AGROW>
                        allStructs{end, 2} = structString; 
                        allStructs{end, 3} = deps; 
                        if ~isempty(busObj)
                            busAssignments{end+1, 1} = topicName; %#ok<AGROW>
                            busAssignments{end, 2} = busObj; 
                        end

                        % Preserve variant metadata and field metadata in the persistent cache.
                        if isempty(topicVariants)
                            topicVariants = {topicName};
                        end

                        existingEntry = struct();
                        if isfield(obj.OrbCache.topics, topicName)
                            existingEntry = obj.OrbCache.topics.(topicName);
                        end

                        % Store variants as cell array of strings
                        topicEntry = struct();
                        topicEntry.variants = topicVariants;
                        if ~isempty(fieldMeta) && height(fieldMeta) > 0
                            % Convert field metadata table to JSON-serializable array of structs
                            fieldsArray = {};
                            for fIdx = 1:height(fieldMeta)
                                fieldsArray{end+1} = struct( ...
                                    'name', fieldMeta.fieldName{fIdx}, ...
                                    'type', fieldMeta.fieldType{fIdx}, ...
                                    'arraySize', fieldMeta.arraySize(fIdx) ...
                                ); %#ok<AGROW>
                            end
                            topicEntry.fields = {fieldsArray};
                        end

                        if isfield(existingEntry, 'orb_id')
                            topicEntry.orb_id = existingEntry.orb_id;
                        end
                        obj.OrbCache.topics.(topicName) = topicEntry;
                    end
                catch ME
                    if obj.ShowDebug
                        fprintf('! Skipping/Error in message [%s]: %s\n', topicName, ME.message);
                    end
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
            % For PX4 builds: Use forward declarations only (real headers in implementation files)
            %                Each .cpp file includes only what it needs
            % For local simulation: use our generated struct definitions
            headerStr = sprintf('%s\n#if defined(__PX4_LINUX) || defined(__PX4_POSIX) || defined(__PX4_NUTTX)\n', headerStr);
            headerStr = sprintf('%s// PX4 Build: Forward declarations only (implementations include their own headers)\n', headerStr);
            % Note: struct definitions are in #else branch below

            headerStr = sprintf('%s#else\n', headerStr);
            headerStr = sprintf('%s// Local Simulation: use generated struct definitions\n\n', headerStr);

            % Output all struct definitions (now forward declarations exist)
            % First, sort structs by dependencies to ensure definitions come before usage
            if ~isempty(allStructs)
                allStructs = obj.topologicalSortStructs(allStructs);
            end
            for i = 1:size(allStructs, 1)
                headerStr = sprintf('%s%s\n', headerStr, allStructs{i, 2});
            end

            headerStr = sprintf('%s\n#endif  // End PX4 vs Local struct definitions\n\n', headerStr);

            % Start appending clean, strongly-typed function signatures underneath with C Linkage
            headerStr = sprintf('%s#ifdef __cplusplus\nextern "C" {\n#endif\n\n', headerStr);
            headerStr = sprintf('%s// --- STRONGLY-TYPED RETURN-BY-VALUE PROTOTYPES FOR C CALLER ---\n', headerStr);

            srcStr = sprintf('#include "%s"\n\n', obj.StubHeaderName);

            % For PX4 builds, implementations are in simulink_io_glue.cpp
            % For local simulation, provide empty stubs
            srcStr = sprintf('%s#if !defined(__PX4_LINUX) && !defined(__PX4_POSIX) && !defined(__PX4_NUTTX)\n', srcStr);
            srcStr = sprintf('%s// Local simulation stubs only\n\n', srcStr);

            % Append zero-input reader functions and single-input writer functions for all topics
            % Convert CamelCase filenames to snake_case immediately for consistent naming
            for i = 1:length(msgFiles)
                [~, camelName, ~] = fileparts(msgFiles(i).name);
                topicName = obj.camelCaseToSnakeCase(camelName);  % Convert to snake_case

                % 1. Reader Prototype & Mock Source: Returns full structure layout by value
                headerStr = sprintf('%sstruct %s_s read_%s(void);\n', headerStr, topicName, topicName);
                srcStr = sprintf('%sextern "C" struct %s_s read_%s(void) { struct %s_s empty = {0}; return empty; }\n', srcStr, topicName, topicName, topicName);

                % 2. Writer Prototype & Mock Source: Accepts flat structure layout copy by value
                headerStr = sprintf('%svoid write_%s(struct %s_s in);\n', headerStr, topicName, topicName);
                srcStr = sprintf('%sextern "C" void write_%s(struct %s_s in) {}\n', srcStr, topicName, topicName);

                % 3. Dynamic Message Initialization Prototype & Mock Source
                % Accepts a boolean flag (true for NaN initialization, false for Zero initialization)
                headerStr = sprintf('%sstruct %s_s init_%s(bool initialize_to_nan);\n', headerStr, topicName, topicName);
                srcStr = sprintf('%sextern "C" struct %s_s init_%s(bool initialize_to_nan) { struct %s_s empty = {0}; return empty; }\n', srcStr, topicName, topicName, topicName);
            end

            % Append a strongly-typed, zero-input function that returns system time by value
            headerStr = sprintf('%s\n// --- NATIVE HIGH-RESOLUTION SYSTEM CLOCK INTERFACES ---\n', headerStr);
            headerStr = sprintf('%suint64_t read_px4_system_time(void);\n', headerStr);
            srcStr = sprintf('%sextern "C" uint64_t read_px4_system_time(void) { return 0; }\n', srcStr);

            % =========================================================================
            % NATIVE PARAMETER ENGINE SYSTEM BRIDGES FOR C CALLER
            % =========================================================================
            headerStr = sprintf('%s\n// --- NATIVE LIVE PARAMETER SYSTEM BRIDGES ---\n', headerStr);
            headerStr = sprintf('%sfloat read_px4_param_float(const char* param_name);\n', headerStr);
            headerStr = sprintf('%sint32_t read_px4_param_int32(const char* param_name);\n', headerStr);
            headerStr = sprintf('%svoid write_px4_param_float(const char* param_name, float value);\n', headerStr);
            headerStr = sprintf('%svoid write_px4_param_int32(const char* param_name, int32_t value);\n', headerStr);

            srcStr = sprintf('%sextern "C" float read_px4_param_float(const char* param_name) { return 0.0f; }\n', srcStr);
            srcStr = sprintf('%sextern "C" int32_t read_px4_param_int32(const char* param_name) { return 0; }\n', srcStr);
            srcStr = sprintf('%sextern "C" void write_px4_param_float(const char* param_name, float value) {}\n', srcStr);
            srcStr = sprintf('%sextern "C" void write_px4_param_int32(const char* param_name, int32_t value) {}\n', srcStr);

            srcStr = sprintf('%s#endif\n', srcStr);

            headerStr = sprintf('%s\n#ifdef __cplusplus\n}\n#endif\n\n#endif // PX4_SIMULINK_API_H\n', headerStr);

            % 1. Write out the single combined stub header file locally
            if ~exist(obj.LocalGeneratedDir, 'dir')
                mkdir(obj.LocalGeneratedDir);
            end
            headerFilePath = fullfile(obj.LocalGeneratedDir, obj.StubHeaderName);
            fid = fopen(headerFilePath, 'w');
            if fid == -1
                error('[px4API:Error] Could not write header file: %s', headerFilePath);
            end
            fprintf(fid, '%s', headerStr);
            fclose(fid);
            if obj.ShowDebug
                fprintf('✓ Successfully synchronized concrete header: %s\n', obj.StubHeaderName);
            end

            % 2. Write out the matching strongly-typed source stubs file locally
            srcFilePath = fullfile(obj.LocalGeneratedDir, 'px4_simulink_api.cpp');
            fid = fopen(srcFilePath, 'w');
            if fid == -1
                error('[px4API:Error] Could not write source file: %s', srcFilePath);
            end
            fprintf(fid, '%s', srcStr);
            fclose(fid);
            if obj.ShowDebug
                fprintf('✓ Successfully generated source stubs for C Caller: px4_simulink_api.cpp\n');
            end

            % 3. Assign all Simulink bus objects to the workspace
            % (Now that forward declarations exist, all inter-struct references are valid)
            for i = 1:size(busAssignments, 1)
                topicName = busAssignments{i, 1};
                busObj = busAssignments{i, 2};
                nativeStructName = [topicName, '_s'];
                assignin('base', nativeStructName, busObj);
            end

            if obj.ShowDebug
                fprintf('--- [px4API] System Ready for Simulink Modelling ---\n\n');
            end

            % Persist the comprehensive cache (with both orb_id and field metadata) to JSON
            obj.saveOrbCache();
        end


        function typeSize = getPx4FieldTypeSize(~, px4Type)
            % Match PX4 uORB layout ordering rules for struct packing.
            switch lower(px4Type)
                case {'uint64','int64','float64'}
                    typeSize = 8;
                case {'uint32','int32','float32'}
                    typeSize = 4;
                case {'uint16','int16'}
                    typeSize = 2;
                case {'uint8','int8','bool','char'}
                    typeSize = 1;
                otherwise
                    typeSize = 0;
            end
        end

        function [structStr, busObj, dependencies, fieldMetadata, topicVariants] = generateBusFromMsg(obj, camelName, topicName, msgFilePath)
            % Generate Simulink bus structure from PX4 .msg file
            % Returns: structStr (C struct definition), busObj (Simulink bus), dependencies (list of required structs)
            %          fieldMetadata (table of field info: fieldName, fieldType, arraySize)
            %          topicVariants (cell array of PX4 topic IDs declared by the message)
            % camelName: original CamelCase filename (used to open the file)
            % topicName: snake_case topic name (used for struct naming)
            % msgFilePath: fully-qualified path to the discovered .msg file when scanning subfolders
            %
            % If called with single argument (legacy), assume input is already camelName
            if nargin == 2
                topicName = obj.camelCaseToSnakeCase(camelName);
                msgFilePath = '';
            end

            structStr = '';
            busObj = [];
            dependencies = {};
            fieldMetadata = table();  % Initialize empty table for field metadata

            % Open the discovered file path when available; otherwise fall back to a recursive search.
            if nargin < 4 || isempty(msgFilePath)
                msgFilePath = fullfile(obj.PX4Root, 'msg', [camelName, '.msg']);
                if exist(msgFilePath, 'file') ~= 2
                    fallbackFiles = dir(fullfile(obj.PX4Root, 'msg', '**', '*.msg'));
                    for fallbackIdx = 1:length(fallbackFiles)
                        [~, fallbackCamelName, ~] = fileparts(fallbackFiles(fallbackIdx).name);
                        if strcmp(fallbackCamelName, camelName)
                            msgFilePath = fullfile(fallbackFiles(fallbackIdx).folder, fallbackFiles(fallbackIdx).name);
                            break;
                        end
                    end
                end
            end
            fid = fopen(msgFilePath, 'r');
            if fid == -1
                error('[px4API:Error] Could not open message file: %s', msgFilePath);
            end
            fileData = textscan(fid, '%s', 'Delimiter', '\n');
            fclose(fid);
            lines = fileData{1};

            elements = [];
            structBody = sprintf('struct %s_s {\n', topicName);
            parsedFields = {};

            % Track message variant names from PX4 "#TOPICS" metadata (if present)
            topicVariants = {};

            % Temporary array block to collect dynamic constant configurations
            constantsList = {}; 

            % Initialize arrays to store field metadata
            fieldNames = {};
            fieldTypes = {};
            arraySizes = [];

            for i = 1:length(lines)
                line = strtrim(lines{i});
                if startsWith(line, '#')
                    % Extract topic variants from a TOPICS metadata comment
                    stripped = regexprep(line, '^#\s*', '');
                    toks = strsplit(strtrim(stripped));
                    if ~isempty(toks) && strcmpi(toks{1}, 'TOPICS')
                        newVariants = toks(2:end);
                        newVariants = newVariants(~cellfun(@isempty, newVariants));
                        topicVariants = [topicVariants, newVariants]; %#ok<AGROW>
                    end
                    continue;
                end

                commentIdx = strfind(line, '#');
                if ~isempty(commentIdx)
                    line = strtrim(line(1:commentIdx(1)-1));
                end

                % Looks for value assignments like: uint8 ACTION_ARM = 1
                if contains(line, '=')
                    tokens = strsplit(line, '=');
                    if length(tokens) >= 2
                        leftSide = strtrim(tokens{1});
                        rightSide = strtrim(tokens{2});
                        
                        % Split type from name on the left side (e.g. "uint8 ACTION_ARM")
                        typeAndName = strsplit(leftSide);
                        if length(typeAndName) >= 2
                            constName = typeAndName{2};
                            constVal = rightSide;
                            
                            % Strip trailing semicolons if present in metadata configs
                            if endsWith(constVal, ';'), constVal = constVal(1:end-1); end
                            
                            constantsList{end+1, 1} = constName; %#ok<AGROW>
                            constantsList{end, 2} = constVal;     
                        end
                    end
                    continue; % Safe filter: Constants are completely skipped from structural field additions
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
                    depNameSnake = obj.camelCaseToSnakeCase(depName);
                    if ~any(strcmp(dependencies, depNameSnake))
                        dependencies{end+1} = depNameSnake; %#ok<AGROW>
                    end
                end

                parsedFields{end+1} = struct('px4Type', px4Type, 'varName', varName, ...
                    'arraySize', arraySize, 'cType', cType, 'slType', slType, ...
                    'isDependency', isDependency, 'depName', depName); %#ok<AGROW>
            end

            % Reorder fields by native PX4 uORB size so the generated layout matches
            % the runtime topic struct used by orb_copy()/orb_publish().
            if ~isempty(parsedFields)
                fieldSizes = cellfun(@(f) obj.getPx4FieldTypeSize(f.px4Type), parsedFields);
                [~, order] = sort(fieldSizes, 'descend');
                parsedFields = parsedFields(order);


                % Extract metadata from the REORDERED parsedFields. This ensures that 
                % the JSON cache and the Simulink Bus match the C struct layout exactly.
                parsedFieldsStruct = [parsedFields{:}];  % Convert cell to struct array
                fieldNames = {parsedFieldsStruct.varName};
                fieldTypes = {parsedFieldsStruct.px4Type};
                arraySizes = [parsedFieldsStruct.arraySize];
            end

            % Emit the struct in the matched layout order.
            for i = 1:numel(parsedFields)
                f = parsedFields{i};
                if f.arraySize > 1
                    structBody = sprintf('%s    %s %s[%d];\n', structBody, f.cType, f.varName, f.arraySize);
                else
                    structBody = sprintf('%s    %s %s;\n', structBody, f.cType, f.varName);
                end

                elem = Simulink.BusElement;
                elem.Name = f.varName;
                elem.DataType = f.slType;
                elem.Dimensions = f.arraySize;
                elem.Complexity = 'real';
                elements = [elements; elem]; %#ok<AGROW>
            end

            % Normalize topic variants in case multiple #TOPICS lines were present
            if ~isempty(topicVariants)
                topicVariants = unique(topicVariants, 'stable');
            end

            % If constants exist for this message, dynamically compile a native 
            % Simulink integer enumeration file directly into the LocalGeneratedDir.
            if ~isempty(constantsList)
                enumFileName = fullfile(obj.PackageRoot, '+enums', [topicName '.m']);
                efid = fopen(enumFileName, 'w');
                if efid ~= -1
                    fprintf(efid, 'classdef %s < Simulink.IntEnumType\n', topicName);
                    fprintf(efid, '    enumeration\n');
                    for cIdx = 1:size(constantsList, 1)
                        fprintf(efid, '        %s(%s)\n', constantsList{cIdx, 1}, constantsList{cIdx, 2});
                    end
                    fprintf(efid, '    end\n');
                    fprintf(efid, 'end\n');
                    fclose(efid);
                    clear(topicName); % Clear class definition cache so MATLAB picks up changes instantly
                end
            end

            if ~isempty(parsedFields)
                % Preserve the exact lowercase snake_case name of the message file
                nativeStructName = [lower(topicName), '_s'];

                structBody = sprintf('%s};\ntypedef struct %s %s;\n', structBody, nativeStructName, nativeStructName);
                structStr = structBody;

                % OPTIMIZATION: Create bus object using the shared helper method
                busObj = obj.createBusFromFieldData(parsedFields);

                % Populate field metadata table for caching in JSON
                fieldMetadata = table(fieldNames', fieldTypes', arraySizes', ...
                    'VariableNames', {'fieldName', 'fieldType', 'arraySize'});
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
                obj.generateAllBussesAndHeaders();
            end

            % Extract model component name natively from buildInfo
            modelName = buildInfo.ComponentName;
            if isempty(modelName)
                error('[px4API:Error] buildInfo must provide ComponentName.');
            end

            % Generate model-agnostic wrapper header (decouples PX4 code from model name)
            obj.generateModelWrapper(modelName, obj.LocalGeneratedDir);

            % Generate the model-specific glue locally from the live Simulink model.
            obj.generateOmnipotentCppGlue(obj.LocalGeneratedDir, modelName);
            filesExportedCount = 0;
            filesExportedCount = filesExportedCount + obj.copyGeneratedCodeFiles(obj.LocalGeneratedDir, obj.ResolvedExternalDir, false);
            
            % Extract the model name directly from the buildInfo metadata token
            modelName = buildInfo.ComponentName;
            
            % Retrieves a structure containing
            % the absolute paths to the code generation output targets securely.
            buildDirInfo = RTW.getBuildDir(modelName);
            buildDir = buildDirInfo.BuildDirectory;

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

                    % Skip px4_simulink_api.cpp for PX4 builds (empty/simulation-only stubs)
                    if strcmp(entries(i).name, 'px4_simulink_api.cpp')
                        continue;
                    end

                    if ismember(lower(ext), extAllowed)
                        copyfile(sourcePath, destPath, 'f');
                        count = count + 1;
                    end
                end
            end
        end

        function generateOmnipotentCppGlue(obj, outputDir, modelName)
            % Generate omnipotent uORB routing layer and parameter access functions.
            %
            % This is the second major code generation stage, creating:
            %   1. simulink_io_glue.cpp - Implementation of read/write/init functions
            %   2. Parameter access layer - read_px4_param_* and write_px4_param_*
            %
            % Optimization strategy:
            %   - Selective generation: Only generates code for topics used in the model
            %   - Scans Test_ert_rtw/Test.cpp to find actual function calls
            %   - Uses regex to identify read_*/write_* calls (excludes px4_param_* functions)
            %   - Reduces compile time and binary size vs. generating for all 200+ topics
            %
            % Parameter handling:
            %   - write_* functions check if value changed before calling param_set
            %   - Avoids unnecessary system notifications and syncs
            %   - Uses 1e-6f epsilon for float comparisons (matches PX4 FLT_EPSILON)
            %
            % Input:
            %   outputDir - Directory for simulink_io_glue.cpp (default: ResolvedExternalDir)
            %
            % Generated file structure:
            %   1. Standard includes (cmath, uORB, etc.)
            %   2. extern "C" { ... }      <- uORB topic functions (read/write/init)
            %   3. } // extern "C"
            %   4. #include <parameters/param.h>     <- Avoid C linkage for C++ headers
            %   5. extern "C" { ... }      <- Parameter functions
            %   6. }

            if nargin < 2 || isempty(outputDir)
                outputDir = obj.ResolvedExternalDir;
            end
            if obj.ShowDebug
                fprintf('\n--- [px4API] Generating Selective C++ uORB Glue Code (functions used only) ---\n');
            end

            % Step 1: Scan generated model code to find which functions are called
            requiredTopics = {};
            testCppPath = fullfile(obj.PackageRoot, sprintf('%s_ert_rtw', modelName), sprintf('%s.cpp', modelName));
            if isfile(testCppPath)
                fid = fopen(testCppPath, 'r');
                testContent = fread(fid, '*char')';
                fclose(fid);

                % Find all read_TOPIC() and write_TOPIC() calls, but keep PX4 parameter
                % helpers out of the uORB topic list.
                readMatches = regexp(testContent, 'read_(?!px4_param_)(\w+)\s*\(', 'tokens');
                writeMatches = regexp(testContent, 'write_(?!px4_param_)(\w+)\s*\(', 'tokens');

                % FIXED MATRICES BLOCK: Force convert both results into linear row cells
                % to prevent dimension mismatch crashes regardless of argument contents.
                readTopicsList = {};
                if ~isempty(readMatches), readTopicsList = [readMatches{:}]; end

                writeTopicsList = {};
                if ~isempty(writeMatches), writeTopicsList = [writeMatches{:}]; end

                % Combine the flat arrays horizontally safely
                allMatches = [readTopicsList, writeTopicsList];
                if ~isempty(allMatches)
                    requiredTopics = unique(allMatches);
                end

                % Separate special non-uORB topics from regular uORB topics
                hasSystemTime = false;
                orbTopics = {};
                for i = 1:length(requiredTopics)
                    if strcmp(requiredTopics{i}, 'px4_system_time')
                        hasSystemTime = true;
                    else
                        orbTopics{end+1} = requiredTopics{i}; %#ok<AGROW>
                    end
                end

                if obj.ShowDebug
                    fprintf('  Found %d unique topics used in model\n', length(requiredTopics));
                end                
            else
                if obj.ShowDebug
                    fprintf('  Warning: Could not find Test.cpp, generating for all topics and methods\n');
                end
                % obj.OrbCache.topics %%% We have this instead and should use the cache!
                orbTopics = fieldnames(obj.OrbCache.topics)';
                hasSystemTime = true;
            end

            % Start building the C++ source file
            cppStr = sprintf('// Auto-generated selective strongly-typed return-by-value uORB routing layer\n');
            cppStr = sprintf('%s#include <cmath>\n', cppStr); % Standard C++ math header for NAN definitions
            cppStr = sprintf('%s#include <px4_platform_common/defines.h>\n', cppStr); % Core PX4 macro platform propert
            cppStr = sprintf('%s#include <px4_platform_common/log.h>\n#include <uORB/uORB.h>\n', cppStr);
            cppStr = sprintf('%s#include <string.h>\n', cppStr);
            cppStr = sprintf('%s#include "px4_simulink_api.h"\n', cppStr);

            % Include hrt header if system time is needed (high-resolution timer)
            if hasSystemTime
                cppStr = sprintf('%s#include <drivers/drv_hrt.h>\n', cppStr);
            end

            % Include real uORB topic headers for glue function implementations
            for i = 1:length(orbTopics)
                topicName = orbTopics{i};
                cppStr = sprintf('%s#include <uORB/topics/%s.h>\n', cppStr, topicName);
            end

            % --- THE LINUX LINKER FIXED PASS ---
            % Enforce pure C linkage output rules for ALL generated implementation blocks.
            % This prevents the C++ compiler from mangling function signatures, which resolves
            % the "undefined reference" errors during the final bin/px4 link step.
            cppStr = sprintf('%s\nextern "C" {\n\n', cppStr);

            % =========================================================================
            % SPECIAL CASE: System Time (uses hrt_absolute_time, not uORB)
            % =========================================================================
            if hasSystemTime
                cppStr = sprintf('%suint64_t read_px4_system_time(void) {\n', cppStr);
                cppStr = sprintf('%s    return hrt_absolute_time();\n', cppStr);
                cppStr = sprintf('%s}\n\n', cppStr);
            end

            % =========================================================================
            % 1. RETURN-BY-VALUE READER FUNCTIONS (for required uORB topics only)
            % =========================================================================
            % Returning the structure directly by value forces the Simulink C Caller to
            % recognize the function as a pure output node, removing the dual out_buffer ports.
            for i = 1:length(orbTopics)
                baseTopic = orbTopics{i};
                variants = obj.getTopicVariants(baseTopic);
                for v = 1:length(variants)
                    variantName = variants{v};
                    cppStr = sprintf('%sstruct %s_s read_%s(void) {\n', cppStr, baseTopic, variantName);
                    cppStr = sprintf('%s    static int sub_handle = -1;\n', cppStr);
                    cppStr = sprintf('%s    if (sub_handle < 0) { sub_handle = orb_subscribe(ORB_ID(%s)); }\n', cppStr, variantName);
                    cppStr = sprintf('%s    static struct %s_s local_buffer;\n', cppStr, baseTopic);
                    cppStr = sprintf('%s    bool updated = false;\n', cppStr);
                    cppStr = sprintf('%s    orb_check(sub_handle, &updated);\n', cppStr);
                    cppStr = sprintf('%s    if (updated) { orb_copy(ORB_ID(%s), sub_handle, &local_buffer); }\n', cppStr, variantName);
                    cppStr = sprintf('%s    return local_buffer;\n}\n\n', cppStr);
                end
                if ~any(strcmp(variants, baseTopic))
                    cppStr = sprintf('%sstruct %s_s read_%s(void) {\n', cppStr, baseTopic, baseTopic);
                    cppStr = sprintf('%s    return read_%s();\n}\n\n', cppStr, variants{1});
                end
            end

            % =========================================================================
            % 2. PASS-BY-VALUE WRITER FUNCTIONS (for required uORB topics only)
            % =========================================================================
            % Accepting the struct copy directly by value natively places a single input port
            % arrow on the left-hand face of the C Caller block without triggering parameter scope leaks.
            for i = 1:length(orbTopics)
                baseTopic = orbTopics{i};
                variants = obj.getTopicVariants(baseTopic);
                for v = 1:length(variants)
                    variantName = variants{v};
                    cppStr = sprintf('%svoid write_%s(struct %s_s in) {\n', cppStr, variantName, baseTopic);
                    cppStr = sprintf('%s    static orb_advert_t pub_handle = nullptr;\n', cppStr);
                    cppStr = sprintf('%s    if (pub_handle == nullptr) { pub_handle = orb_advertise(ORB_ID(%s), &in); }\n', cppStr, variantName);
                    cppStr = sprintf('%s    else { orb_publish(ORB_ID(%s), pub_handle, &in); }\n', cppStr, variantName);
                    cppStr = sprintf('%s}\n\n', cppStr);
                end
                if ~any(strcmp(variants, baseTopic))
                    cppStr = sprintf('%svoid write_%s(struct %s_s in) {\n', cppStr, baseTopic, baseTopic);
                    cppStr = sprintf('%s    write_%s(in);\n}\n\n', cppStr, variants{1});
                end
            end

            % =========================================================================
            % 3. DYNAMIC INITIALIZATION FUNCTIONS (for required uORB topics only)
            % =========================================================================
            % Zeroes out integers/booleans natively and maps the NAN compiler macro
            % dynamically across only the verified floating-point fields (float32/float64).
            % Uses cached field metadata from persistent JSON cache.
            for i = 1:length(orbTopics)
                baseTopic = orbTopics{i};
                variants = obj.getTopicVariants(baseTopic);
                for v = 1:length(variants)
                    variantName = variants{v};
                    cppStr = sprintf('%sstruct %s_s init_%s(bool initialize_to_nan) {\n', cppStr, baseTopic, variantName);
                    cppStr = sprintf('%s    struct %s_s msg;\n', cppStr, baseTopic);
                    cppStr = sprintf('%s    memset(&msg, 0, sizeof(msg));\n', cppStr);
                    cppStr = sprintf('%s    if (initialize_to_nan) {\n', cppStr);

                    % Retrieve field metadata from JSON cache
                    fieldMeta = obj.getFieldMetadataFromCache(baseTopic);

                    % Generate NaN initialization code from cached metadata
                    if ~isempty(fieldMeta) && height(fieldMeta) > 0
                        % Iterate over all fields in the table
                        for fieldIdx = 1:height(fieldMeta)
                            fieldType = fieldMeta.fieldType{fieldIdx};
                            % Determine if this is a floating-point type (only these need NaN init)
                            isFloat = strcmp(fieldType, 'float32') || strcmp(fieldType, 'float64');

                            if isFloat
                                fieldName = fieldMeta.fieldName{fieldIdx};
                                arraySize = fieldMeta.arraySize(fieldIdx);

                                if arraySize > 1
                                    % Handle array fields by looping the macro assignment
                                    for idx = 0:(arraySize-1)
                                        cppStr = sprintf('%s        msg.%s[%d] = NAN;\n', cppStr, fieldName, idx);
                                    end
                                else
                                    % Single element field assignment
                                    cppStr = sprintf('%s        msg.%s = NAN;\n', cppStr, fieldName);
                                end
                            end
                        end
                    end

                    cppStr = sprintf('%s    }\n', cppStr);
                    cppStr = sprintf('%s    return msg;\n}\n\n', cppStr);
                end
                if ~any(strcmp(variants, baseTopic))
                    cppStr = sprintf('%sstruct %s_s init_%s(bool initialize_to_nan) {\n', cppStr, baseTopic, baseTopic);
                    cppStr = sprintf('%s    return init_%s(initialize_to_nan);\n}\n\n', cppStr, variants{1});
                end
            end

            % =========================================================================
            % 5. HIGH-SPEED RUNTIME VALIDATION PARAMETER LAYER
            % =========================================================================
            % Close the C linkage block so C++ headers can declare C++ linkage symbols.
            cppStr = sprintf('%s\n} // extern "C"\n\n', cppStr);
            cppStr = sprintf('%s// --- OPTIMIZED CACHED PARAMETER ACCESS LAYER ---\n', cppStr);
            cppStr = sprintf('%s#include <parameters/param.h>\n', cppStr); % Native PX4 parameter system header
            cppStr = sprintf('%s\nextern "C" {\n\n', cppStr); % Reopen C linkage for parameter functions

            % RUNTIME VALIDATION FLOAT READER
            cppStr = sprintf('%s__attribute__((used)) float read_px4_param_float(const char* param_name) {\n', cppStr);
            cppStr = sprintf('%s    param_t handle = param_find(param_name);\n', cppStr);
            cppStr = sprintf('%s    if (handle != PARAM_INVALID && param_type(handle) == PARAM_TYPE_FLOAT) {\n', cppStr);
            cppStr = sprintf('%s        float val = NAN;\n', cppStr);
            cppStr = sprintf('%s        if (param_get(handle, &val) == 0) { return val; }\n', cppStr);
            cppStr = sprintf('%s    }\n', cppStr);
            cppStr = sprintf('%s    return NAN;\n}\n\n', cppStr);

            cppStr = sprintf('%s__attribute__((used)) int32_t read_px4_param_int32(const char* param_name) {\n', cppStr);
            cppStr = sprintf('%s    param_t handle = param_find(param_name);\n', cppStr);
            cppStr = sprintf('%s    if (handle != PARAM_INVALID && param_type(handle) == PARAM_TYPE_INT32) {\n', cppStr);
            cppStr = sprintf('%s        int32_t val = 0;\n', cppStr);
            cppStr = sprintf('%s        if (param_get(handle, &val) == 0) { return val; }\n', cppStr);
            cppStr = sprintf('%s    }\n', cppStr);
            cppStr = sprintf('%s    return 0;\n}\n\n', cppStr);

            % RUNTIME VALIDATION WRITERS
            % Only update if value actually changed to avoid unnecessary notifications and syncs
            cppStr = sprintf('%s__attribute__((used)) void write_px4_param_float(const char* param_name, float value) {\n', cppStr);
            cppStr = sprintf('%s    param_t handle = param_find(param_name);\n', cppStr);
            cppStr = sprintf('%s    if (handle != PARAM_INVALID && param_type(handle) == PARAM_TYPE_FLOAT) {\n', cppStr);
            cppStr = sprintf('%s        float current_val = 0.0f;\n', cppStr);
            cppStr = sprintf('%s        if (param_get(handle, &current_val) == 0) {\n', cppStr);
            cppStr = sprintf('%s            if (fabsf(current_val - value) > 1e-6f) {  // FLT_EPSILON-like comparison\n', cppStr);
            cppStr = sprintf('%s                param_set(handle, &value);\n', cppStr);
            cppStr = sprintf('%s            }\n', cppStr);
            cppStr = sprintf('%s        }\n', cppStr);
            cppStr = sprintf('%s    }\n}\n\n', cppStr);

            cppStr = sprintf('%s__attribute__((used)) void write_px4_param_int32(const char* param_name, int32_t value) {\n', cppStr);
            cppStr = sprintf('%s    param_t handle = param_find(param_name);\n', cppStr);
            cppStr = sprintf('%s    if (handle != PARAM_INVALID && param_type(handle) == PARAM_TYPE_INT32) {\n', cppStr);
            cppStr = sprintf('%s        int32_t current_val = 0;\n', cppStr);
            cppStr = sprintf('%s        if (param_get(handle, &current_val) == 0) {\n', cppStr);
            cppStr = sprintf('%s            if (current_val != value) {\n', cppStr);
            cppStr = sprintf('%s                param_set(handle, &value);\n', cppStr);
            cppStr = sprintf('%s            }\n', cppStr);
            cppStr = sprintf('%s        }\n', cppStr);
            cppStr = sprintf('%s    }\n}\n\n', cppStr);

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
            if obj.ShowDebug
                fprintf('✓ Successfully wrote omnipotent uORB router: %s\n\n', glueFilePath);
            end
        end
    end

    methods (Static)
        % ========== STATIC HELPER METHODS ==========
        % Type conversion, naming utilities, and topological sorting
        function resolvedPath = resolveAbsolutePath(pathStr)
            % Resolve tilde, relative, and other path formats to absolute paths.
            %
            % This helper normalizes user-provided paths to handle:
            %   - Tilde expansion (~ -> $HOME)
            %   - Relative paths (./ ../)
            %   - Already absolute paths (passed through)
            %
            % Input:
            %   pathStr - Path string (any format)
            %
            % Output:
            %   resolvedPath - Absolute path string

            resolvedPath = pathStr;
            if startsWith(resolvedPath, '~/') || strcmp(resolvedPath, '~')
                resolvedPath = fullfile(getenv('HOME'), resolvedPath(2:end));
            elseif startsWith(resolvedPath, './') || startsWith(resolvedPath, '../')
                resolvedPath = char(java.io.File(resolvedPath).getCanonicalPath());
            end
        end

        function snakeName = camelCaseToSnakeCase(camelName)
            % Convert CamelCase file names to snake_case uORB topic names.
            %
            % PX4 firmware v1.16+ uses CamelCase for .msg filenames but snake_case
            % for internal uORB topic names.
            %
            % Algorithm:
            %   1. Insert underscore before each uppercase letter when preceded by a letter or digit
            %   2. Convert result to lowercase
            %
            % Examples:
            %   SensorAirflow -> sensor_airflow
            %   VehicleAttitude -> vehicle_attitude
            %   Ekf2Timestamps -> ekf2_timestamps

            % FIXED REGEX: ([a-z0-9]) captures digits as well as lowercase letters,
            % forcing the boundary split to occur cleanly on names containing acronym numbers.
            snakeName = regexprep(camelName, '([a-z0-9])([A-Z])', '$1_$2');
            snakeName = lower(snakeName);
        end


        function variants = extractMsgTopicVariants(msgFilePath, defaultTopicName)
            % Read a .msg file and return its explicit TOPICS definition if present.
            %
            % If the file contains a "#TOPICS" line, the returned list is exactly
            % those topic names. Otherwise, the default topic name is returned.
            variants = {};
            if ~exist(msgFilePath, 'file')
                variants = {defaultTopicName};
                return;
            end

            fid = fopen(msgFilePath, 'r');
            if fid == -1
                variants = {defaultTopicName};
                return;
            end

            fileData = textscan(fid, '%s', 'Delimiter', '\n');
            fclose(fid);
            lines = fileData{1};

            for i = 1:length(lines)
                line = strtrim(lines{i});
                if ~startsWith(line, '#')
                    continue;
                end

                stripped = regexprep(line, '^#\s*', '');
                toks = strsplit(strtrim(stripped));
                if ~isempty(toks) && strcmpi(toks{1}, 'TOPICS')
                    newTopics = toks(2:end);
                    newTopics = newTopics(~cellfun(@isempty, newTopics));
                    variants = [variants, newTopics]; %#ok<AGROW>
                end
            end

            variants = unique(variants, 'stable');
            if isempty(variants)
                variants = {defaultTopicName};
            end
        end

        function [cType, slType] = px4TypeToCTypes(px4Type)
            % Map PX4 .msg primitive types to C and Simulink type names.
            %
            % Maps both standard C types (uint32, int32, etc.) and
            % composite types (struct references).
            %
            % Inputs:
            %   px4Type - Type name from .msg file (e.g., 'float32', 'int32', 'bool')
            %
            % Outputs:
            %   cType - C type string (e.g., 'float', 'int32_t', 'struct vehicle_attitude_s')
            %   slType - Simulink type name (e.g., 'single', 'int32', 'vehicle_attitude_s')

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
                    % Non-primitive: assume embedded message type (struct reference)
                    slType = [px4Type, '_s'];
                    cType = ['struct ', px4Type, '_s'];
            end
        end

        function [cType, slType, isDependency, depName] = px4TypeToCTypesWithDeps(px4Type)
            % Map PX4 types to C/Simulink types and detect struct dependencies.
            %
            % Used during code generation to:
            %   1. Generate correct C struct members
            %   2. Detect embedded message types (struct references)
            %   3. Track dependencies for topological sorting
            %
            % Outputs:
            %   isDependency - true if this field references another struct
            %   depName - CamelCase name of the referenced struct (if isDependency=true)

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
                    % Non-primitive: embedded message type (struct reference)
                    isDependency = true;
                    depName = px4Type;  % Store CamelCase name for dependency tracking
                    % Convert struct name to snake_case for C type
                    structNameSnake = px4io.px4API.camelCaseToSnakeCase(px4Type);
                    slType = [structNameSnake, '_s'];
                    cType = ['struct ', structNameSnake, '_s'];
            end
        end

        function orderedStructs = topologicalSortStructs(allStructs)
            % Topologically sort struct definitions by dependencies.
            %
            % Ensures structs are defined before they're referenced, avoiding
            % "incomplete type" compilation errors.
            %
            % Input:
            %   allStructs - Cell array {topicName, structStr, dependencies}
            %                where dependencies is a cell array of topic names
            %
            % Output:
            %   orderedStructs - Cell array {topicName, structStr} sorted with
            %                    dependencies appearing before dependents
            %
            % Algorithm: Depth-first traversal with cycle detection

            if isempty(allStructs)
                orderedStructs = {};
                return;
            end

            numStructs = size(allStructs, 1);
            struct_map = containers.Map();  % topicName -> index
            for i = 1:numStructs
                struct_map(allStructs{i, 1}) = i;
            end

            % Track visited nodes for DFS
            visited = false(numStructs, 1);
            orderedStructs = {};

            % Depth-first traversal to build dependency order
            for i = 1:numStructs
                if ~visited(i)
                    [orderedStructs, visited] = px4io.px4API.dfs_visit(i, allStructs, struct_map, visited, orderedStructs);
                end
            end
        end


        function [orderedStructs, visited] = dfs_visit(idx, allStructs, struct_map, visited, orderedStructs)
            % Depth-first search helper for topological sort.
            %
            % Recursively visits dependency nodes before the current node,
            % ensuring proper ordering for C struct definitions.

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
                        [orderedStructs, visited] = px4io.px4API.dfs_visit(depIdx, allStructs, struct_map, visited, orderedStructs);
                    end
                end
            end

            % Add current struct after its dependencies
            orderedStructs{end+1, 1} = topicName; 
            orderedStructs{end, 2} = allStructs{idx, 2}; 
        end


        function listStr = getTopicDropdownString()
            % Generate comma-separated list of all available PX4 message topics.
            %
            % Used by Simulink mask callbacks to populate uORB topic selector dropdowns.
            %
            % Output:
            %   listStr - Comma-separated topic names (snake_case, alphabetically sorted)
            %
            % Note: Creates a temporary px4API instance to scan PX4 messages

            api = px4io.px4API();
            msgDir = fullfile(api.PX4Root, 'msg');
            if ~exist(msgDir, 'dir')
                error('[px4API:Error] Could not find the mandatory PX4 message root folder directory at: %s', msgDir);
            end

            files = dir(fullfile(msgDir, '**', '*.msg'));
            topics = {};
            for i = 1:length(files)
                [~, camelName, ~] = fileparts(files(i).name);
                topicName = obj.camelCaseToSnakeCase(camelName);

                % Skip internal metadata framework tags
                if strcmp(topicName, 'message_version'), continue; end

                variants = px4API.extractMsgTopicVariants(fullfile(files(i).folder, files(i).name), topicName);
                for j = 1:length(variants)
                    topics{end+1} = variants{j}; %#ok<AGROW>
                end
            end

            % Deduplicate across subfolders and sort alphabetically
            topics = unique(topics(~cellfun(@isempty, topics)));
            listStr = strjoin(topics, ',');
        end

        function listStr = getBaseTopicsDropdownString()
            % Generate comma-separated list of UNIQUE base message types (no variants).
            %
            % Used by Simulink mask callbacks to populate the PRIMARY topic selector.
            % Variants are selected separately in a dependent dropdown.
            %
            % Output:
            %   listStr - Comma-separated base topic names (snake_case, alphabetically sorted)

            api = px4io.px4API();
            msgDir = fullfile(api.PX4Root, 'msg');
            if ~exist(msgDir, 'dir')
                error('[px4API:Error] Could not find the mandatory PX4 message root folder directory at: %s', msgDir);
            end

            files = dir(fullfile(msgDir, '**', '*.msg'));
            baseTopics = {};
            for i = 1:length(files)
                [~, camelName, ~] = fileparts(files(i).name);
                topicName = api.camelCaseToSnakeCase(camelName);

                % Skip internal metadata framework tags
                if strcmp(topicName, 'message_version'), continue; end

                % Add only the base topic name, not variants
                baseTopics{end+1} = topicName; %#ok<AGROW>
            end

            % Deduplicate across subfolders and sort alphabetically
            baseTopics = unique(baseTopics(~cellfun(@isempty, baseTopics)));
            listStr = strjoin(baseTopics, ',');
        end

        function busObj = createBusFromFieldData(fieldsData)
            % Helper to create a Simulink.Bus object from an array of field structs.
            % Accepts either parsedFields (from .msg parsing) or cached fieldsData (from JSON).
            %
            % Input:
            %   fieldsData - Array of structs containing field definitions.
            %                Expected fields: 
            %                - From cache: name, type, arraySize
            %                - From parser: varName, slType, arraySize

            if isempty(fieldsData)
                busObj = [];
                return;
            end

            elements = [];
            for fIdx = 1:length(fieldsData)
                if iscell(fieldsData)
                    f = fieldsData{fIdx};
                else
                    f = fieldsData(fIdx);
                end

                % Handle both parsedFields format (varName, slType) and cache format (name, type)
                if isfield(f, 'varName')
                    fName = f.varName;
                    fType = f.slType;
                else
                    fName = f.name;
                    [~, fType, ~, ~] = px4io.px4API.px4TypeToCTypesWithDeps(f.type);
                end

                fDim = f.arraySize;

                elem = Simulink.BusElement;
                elem.Name = fName;
                elem.DataType = fType;
                elem.Dimensions = fDim;
                elem.Complexity = 'real';
                elem.SampleTime = -1;       % Inherited sample time
                elem.DimensionsMode = 'Fixed';
                elements = [elements; elem]; %#ok<AGROW>
            end

            busObj = Simulink.Bus;
            busObj.Elements = elements;
        end

        function runPostCodeGen(buildInfo)
            % Simulink callback hook executed immediately after code generation.
            %
            % This is called by Simulink's post-code generation system to:
            %   - Export generated artifacts to PX4 (optional)
            %   - Perform integration checks
            %
            % Input:
            %   buildInfo - Simulink build info structure (contains model metadata)

            apiInstance = px4io.px4API();
            apiInstance.exportGeneratedCode(buildInfo);
        end


        function uorb_topic_callback(callbackContext)
            % Simulink mask callback for uORB topic parameter.
            %
            % Invoked when the uORB topic block mask is initialized,
            % populating the topic selector dropdown with available topics.
            %
            % Input:
            %   callbackContext - Simulink mask context (contains BlockHandle)

            blockHandle = callbackContext.BlockHandle;
            choices = strsplit(px4API.getTopicDropdownString(), ',');
            set_param(blockHandle, 'TypeOptions_uorb_topic', choices);
        end

        function generateModelWrapper(modelName, outputDir)
            % Generate a model-agnostic wrapper header for PX4 code.
            %
            % This wrapper decouples PX4 code from the specific model name,
            % allowing model renaming without requiring changes to PX4 integration code.
            %
            % Inputs:
            %   modelName - The actual Simulink model name (from buildInfo.ComponentName)
            %   outputDir - Directory where wrapper header will be written
            %
            % Output file: simulink_model_wrapper.h
            % Contains: C++ namespace with generic SimulinkModel class
        
            if isempty(modelName) || ~(ischar(modelName) || isstring(modelName))
                error('[px4API:Error] modelName must be provided and non-empty');
            end
        
            % --- NEW: Determine if the model has any Inports or Outports ---
            isModelLoaded = bdIsLoaded(modelName);
            if ~isModelLoaded
                load_system(modelName);
            end
            
            % Query both root-level interfaces
            inports  = find_system(modelName, 'SearchDepth', 1, 'BlockType', 'Inport');
            outports = find_system(modelName, 'SearchDepth', 1, 'BlockType', 'Outport');
            
            hasInputs  = ~isempty(inports);
            hasOutputs = ~isempty(outports);
            
            % Clean up and close the model if we had to load it explicitly
            if ~isModelLoaded
                close_system(modelName, 0); 
            end
            % --------------------------------------------------------------
        
            wrapperStr = sprintf('// Auto-generated model-agnostic wrapper\n');
            wrapperStr = sprintf('%s// Decouples PX4 code from model name to enable model renaming without PX4 changes\n', wrapperStr);
            wrapperStr = sprintf('%s// Model: %s\n', wrapperStr, modelName);
            wrapperStr = sprintf('%s#pragma once\n\n', wrapperStr);
            wrapperStr = sprintf('%s#include "%s.h"\n\n', wrapperStr, modelName);
        
            wrapperStr = sprintf('%snamespace SimulinkWrapper {\n\n', wrapperStr);
            wrapperStr = sprintf('%s// Generic model wrapper (model-name-agnostic interface)\n', wrapperStr);
            wrapperStr = sprintf('%sclass SimulinkModel {\n', wrapperStr);
            wrapperStr = sprintf('%sprivate:\n', wrapperStr);
            wrapperStr = sprintf('%s    %s _model;\n\n', wrapperStr, modelName);
            wrapperStr = sprintf('%spublic:\n', wrapperStr);
            wrapperStr = sprintf('%s    void initialize() { _model.initialize(); }\n', wrapperStr);
            wrapperStr = sprintf('%s    void step() { _model.step(); }\n', wrapperStr);
            
            % --- DYNAMIC HANDLING: Inputs Interface ---
            if hasInputs
                % Use a non-const reference wrapper so the PX4 side can feed input data
                wrapperStr = sprintf('%s    %s::ExtU_%s_T& getExternalInputs() { return _model.getExternalInputs(); }\n', wrapperStr, modelName, modelName);
            else
                wrapperStr = sprintf('%s    // Model has no root inputs; returning nullptr fallback\n', wrapperStr);
                wrapperStr = sprintf('%s    void* getExternalInputs() { return nullptr; }\n', wrapperStr);
            end
        
            % --- DYNAMIC HANDLING: Outputs Interface ---
            if hasOutputs
                wrapperStr = sprintf('%s    const %s::ExtY_%s_T& getExternalOutputs() { return _model.getExternalOutputs(); }\n', wrapperStr, modelName, modelName);
            else
                wrapperStr = sprintf('%s    // Model has no root outputs; returning nullptr fallback\n', wrapperStr);
                wrapperStr = sprintf('%s    void* getExternalOutputs() { return nullptr; }\n', wrapperStr);
            end
            % -------------------------------------------
            
            wrapperStr = sprintf('%s};\n\n', wrapperStr);
            wrapperStr = sprintf('%s}  // namespace SimulinkWrapper\n', wrapperStr);
        
            % Write wrapper header
            if ~exist(outputDir, 'dir')
                mkdir(outputDir);
            end
        
            wrapperPath = fullfile(outputDir, 'simulink_model_wrapper.h');
            fid = fopen(wrapperPath, 'w');
            if fid == -1
                error('[px4API:Error] Could not write wrapper header: %s', wrapperPath);
            end
            fprintf(fid, '%s', wrapperStr);
            fclose(fid);
            fprintf('✓ Generated model-agnostic wrapper: simulink_model_wrapper.h (model: %s)\n', modelName);
        end

    end
end
