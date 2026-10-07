% /****************************************************************************
%  *
%  *    Copyright (C) 2026  Yevhenii Kovryzhenko. All rights reserved.
%  *
%  *    This program is free software: you can redistribute it and/or modify
%  *    it under the terms of the GNU Affero General Public License as published by
%  *    the Free Software Foundation, either version 3 of the License, or
%  *    (at your option) any later version.
%  *
%  *    This program is distributed in the hope that it will be useful,
%  *    but WITHOUT ANY WARRANTY; without even the implied warranty of
%  *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%  *    GNU Affero General Public License Version 3 for more details.
%  *
%  *    You should have received a copy of the
%  *    GNU Affero General Public License Version 3
%  *    along with this program.  If not, see <https://www.gnu.org/licenses/>.
%  *
%  *    1. Redistributions of source code must retain the above copyright
%  *       notice, this list of conditions, and the following disclaimer.
%  *    2. Redistributions in binary form must reproduce the above copyright
%  *       notice, this list of conditions, and the following disclaimer in
%  *       the documentation and/or other materials provided with the
%  *       distribution.
%  *    3. No ownership or credit shall be claimed by anyone not mentioned in
%  *       the above copyright statement.
%  *    4. Any redistribution or public use of this software, in whole or in part,
%  *       whether standalone or as part of a different project, must remain
%  *       under the terms of the GNU Affero General Public License Version 3,
%  *       and all distributions in binary form must be accompanied by a copy of
%  *       the source code, as stated in the GNU Affero General Public License.
%  *
%  ****************************************************************************/

% px4API - Generate the Simulink/PX4 message and parameter interface.
%
% The static C API exposes all discovered messages to Simulink C Caller blocks.
% The post-code-generation callback emits model-specific hardware glue and
% exports the completed build into the configured PX4 module directory.
%
% Parameter bindings come from model blocks. Generated readers use cached
% values; initialization resolves PX4 handles and update notifications refresh
% the cache. Parameter writes still call the PX4 parameter API.
%
% Usage:
%   api = px4io.px4API.getInstance();  % Initialize or reuse the editor-time API.
%   px4io.px4API.runPostCodeGen(buildInfo);  % Model PostCodeGenCommand callback.
% Model settings are managed separately by px4io.deployment.configureModel.

classdef px4API < handle
    properties
        % ===== USER CONFIGURATION =====
        % Absolute path to PX4 firmware repository root
        PX4Root = fullfile('~', 'PX4', 'v1.17.0-mod')

        % Name of target PX4 module (directory in src/modules/)
        PX4ModuleName = 'simulink_io'

        % Supported file extensions for export to PX4 tree
        AllowedExtensions = {'.c', '.cpp', '.h'}        

        % Enable/disable debug console output
        ShowDebug = false
    end

    properties (Access = private)
        PackageRoot = ''            % Absolute path to MATLAB +px4io package directory
        ResolvedExternalDir = ''    % Target PX4 generated code directory
        LocalGeneratedDir = ''      % Local MATLAB generated artifacts directory
        EnumsDir = '';              % Local directory for Simulink IntEnumType files
        
        StubHeaderName = 'px4_simulink_api.h' % Static C API header for Simulink
        StubSrcName = 'px4_simulink_api.c'    % Desktop simulation stubs (excluded from PX4)
        GlueName = 'px4_simulink_glue.cpp'    % PX4 hardware uORB/Param routing layer
        
        OrbCache = struct()         % In-memory cache of uORB topic metadata
        OrbCacheLoaded = false      % Flag indicating if cache was loaded from disk
        OrbCacheFile = 'orb_id_cache.json' % Persistent cache filename
        ParamBindingsFile = 'param_bindings.json' % Per-model authoritative parameter manifest
    end
    
    methods
        function obj = px4API()
            % PX4API - Constructor and initialization engine.
            %
            % Resolves all paths, checks file timestamps, and triggers code 
            % regeneration if PX4 .msg files or generator scripts have changed.
            % If files are up-to-date, it simply restores Simulink Bus objects 
            % into the MATLAB base workspace from the JSON cache.
            
            currentFilePath = mfilename('fullpath');
            [obj.PackageRoot, ~, ~] = fileparts(currentFilePath);
            obj.PX4Root = obj.resolveAbsolutePath(obj.PX4Root);
            obj.ResolvedExternalDir = fullfile(obj.PX4Root, 'src', 'modules', obj.PX4ModuleName, 'generated_code');

            if obj.ShowDebug, fprintf('\n--- [px4API] Initializing ---\n'); end            

            [needsGeneration, generationReason] = obj.needsGeneration();
            if needsGeneration
                if obj.ShowDebug, fprintf('! Regenerating: %s\n', generationReason); end
                obj.generateAllBussesAndHeaders();
            else
                if obj.ShowDebug, fprintf('✓ Artifacts up-to-date. (%s)\n', generationReason); end
                obj.loadOrbCacheFromJson();
                obj.regenerateBusesInWorkspace();
            end
        end

        function regenerateBusesInWorkspace(obj)
            % REGENERATEBUSESINWORKSPACE - Restores Simulink Bus objects to the base workspace.
            %
            % Simulink Bus objects are in-memory MATLAB objects and do not persist 
            % across workspace clears or MATLAB restarts. This method rebuilds them 
            % instantly from the persistent JSON cache without re-parsing .msg files.
            % It also explicitly assigns all topic variants to the workspace to 
            % prevent "DataType not in scope" errors in Simulink C Caller blocks.
            
            if ~obj.OrbCacheLoaded || ~isfield(obj.OrbCache, 'topics')
                return;
            end
            
            topicsList = fieldnames(obj.OrbCache.topics);
            if isempty(topicsList)
                return;
            end

            for idx = 1:length(topicsList)
                tName = topicsList{idx};
                topicData = obj.OrbCache.topics.(tName);
                
                if isfield(topicData, 'fields') && ~isempty(topicData.fields)
                    busObj = obj.createBusFromFieldData(topicData.fields);
                    if ~isempty(busObj)
                        % Assign the base topic bus to the workspace
                        assignin('base', [tName, '_s'], busObj);
                        
                        % Assign all variants to the workspace to ensure they are in scope
                        if isfield(topicData, 'variants')
                            variants = topicData.variants;
                            
                            % Handle MATLAB jsondecode quirks (char vs cell)
                            if ischar(variants) || isstring(variants)
                                variants = {variants};
                            elseif iscell(variants)
                                variants = variants(~cellfun(@isempty, variants));
                            else
                                variants = {variants};
                            end
                            
                            for v = 1:length(variants)
                                vName = variants{v};
                                if (ischar(vName) || isstring(vName)) && ~strcmp(vName, tName)
                                    assignin('base', [vName, '_s'], busObj);
                                end
                            end
                        end
                    end
                end
            end
        end

        function ensureBusesInWorkspace(obj)
            % ENSUREBUSESINWORKSPACE - Restore cached Bus objects after a model
            % close, workspace clear, or MATLAB-side cache eviction. The cheap
            % sentinel check lets mask callbacks share one API without repeatedly
            % rebuilding every Bus object.
            if ~obj.OrbCacheLoaded
                obj.loadOrbCacheFromJson();
            end
            if ~obj.OrbCacheLoaded || ~isfield(obj.OrbCache, 'topics')
                return;
            end

            topics = fieldnames(obj.OrbCache.topics);
            if isempty(topics)
                return;
            end
            sentinelName = [topics{1}, '_s'];
            existsInBase = evalin('base', sprintf('exist(''%s'', ''var'') == 1', sentinelName));
            if ~existsInBase
                obj.regenerateBusesInWorkspace();
            end
        end
        
        function registerParamBinding(obj, blockHandle, direction)
            % REGISTERPARAMBINDING - Cheap editor-time registration for one block.
            %
            % This records one block without scanning the complete model.
            % Deleted/renamed blocks are pruned by syncParamBindings at build time.
            if nargin < 3 || isempty(direction)
                direction = '';
            end

            binding = obj.getParamBindingFromBlock(blockHandle, direction);
            if isempty(binding)
                return;
            end

            bindings = obj.loadParamBindings();
            % A mask edit can change the parameter name or type. Remove this
            % block's previous editor-time registration before merging its new
            % configuration; registrations belonging to other blocks remain.
            bindings = obj.removeBlockFromParamBindings(bindings, obj.blockToPath(blockHandle));
            bindings = obj.mergeParamBinding(bindings, binding);
            obj.saveParamBindings(bindings);
            obj.writeParamArtifacts();
        end

        function syncParamBindings(obj, modelName)
            % SYNCPARAMBINDINGS - Rebuild the manifest from the current model.
            %
            % Scan the root and referenced models, then replace the manifest.
            % Bindings for deleted blocks disappear. This scans subsystem blocks;
            % it does not infer parameter usage from generated C symbols.
            if nargin < 2 || isempty(modelName)
                modelName = bdroot;
            end

            if isempty(modelName)
                return;
            end

            models = find_mdlrefs(modelName, 'KeepModelsLoaded', true);
            blocks = {};
            for modelIndex = 1:numel(models)
                found = find_system(models{modelIndex}, 'LookUnderMasks', 'on', 'FollowLinks', 'on', 'BlockType', 'SubSystem');
                blocks = [blocks; found(:)];
            end
            bindings = struct('px4_name', {}, 'symbol', {}, 'type', {}, ...
                'read', {}, 'write', {}, 'blocks', {});

            for i = 1:length(blocks)
                blk = blocks{i};
                binding = obj.getParamBindingFromBlock(blk, '');
                if ~isempty(binding)
                    bindings = obj.mergeParamBinding(bindings, binding);
                end
            end

            obj.saveParamBindings(bindings);
            obj.writeParamArtifacts();
        end

        function bindings = loadParamBindings(obj)
            bindings = struct('px4_name', {}, 'symbol', {}, 'type', {}, ...
                'read', {}, 'write', {}, 'blocks', {});
            manifestPath = fullfile(obj.LocalGeneratedDir, obj.ParamBindingsFile);
            if ~isfile(manifestPath)
                return;
            end

            try
                manifest = jsondecode(fileread(manifestPath));
                if isfield(manifest, 'bindings') && ~isempty(manifest.bindings)
                    decodedBindings = manifest.bindings;
                    if ~isstruct(decodedBindings)
                        return;
                    end
                    bindings = decodedBindings;
                    if iscell(bindings)
                        bindings = [bindings{:}];
                    end
                end
            catch err
                warning('[px4API] Ignoring unreadable parameter manifest: %s', err.message);
            end
        end

        function saveParamBindings(obj, bindings)
            if ~exist(obj.LocalGeneratedDir, 'dir')
                mkdir(obj.LocalGeneratedDir);
            end

            manifest = struct('schema_version', 1, 'bindings', {bindings});
            newContent = jsonencode(manifest, 'PrettyPrint', true);
            manifestPath = fullfile(obj.LocalGeneratedDir, obj.ParamBindingsFile);
            if isfile(manifestPath) && strcmp(fileread(manifestPath), newContent)
                return;
            end

            % Replace rather than append so stale/deleted bindings disappear. Write
            % through a sibling temporary file to avoid leaving partial JSON behind.
            tempPath = [manifestPath, '.tmp'];
            fid = fopen(tempPath, 'w');
            if fid == -1
                error('[px4API:Error] Could not write parameter manifest: %s', manifestPath);
            end
            fprintf(fid, '%s', newContent);
            fclose(fid);
            [ok, message] = movefile(tempPath, manifestPath, 'f');
            if ~ok
                error('[px4API:Error] Could not replace parameter manifest: %s', message);
            end
        end

        function writeParamArtifacts(obj)
            % WRITEPARAMARTIFACTS - Stable C Caller API, independent of bindings.
            % `name` is a NUL-padded uint8[17] supplied by the hidden ParamName
            % Constant in each mask. PX4 parameter names are at most 16 characters.
            protoStr = sprintf(['float read_param_float(const uint8_t name[17]);\n', ...
                'int32_t read_param_int32(const uint8_t name[17]);\n', ...
                'void write_param_float(const uint8_t name[17], float value);\n', ...
                'void write_param_int32(const uint8_t name[17], int32_t value);\n']);
            implStr = sprintf(['float read_param_float(const uint8_t name[17]) { (void)name; return NAN; }\n', ...
                'int32_t read_param_int32(const uint8_t name[17]) { (void)name; return 0; }\n', ...
                'void write_param_float(const uint8_t name[17], float value) { (void)name; (void)value; }\n', ...
                'void write_param_int32(const uint8_t name[17], int32_t value) { (void)name; (void)value; }\n']);

            hdrPath = fullfile(obj.LocalGeneratedDir, obj.StubHeaderName);
            srcPath = fullfile(obj.LocalGeneratedDir, obj.StubSrcName);

            if ~exist(hdrPath, 'file') || ~exist(srcPath, 'file')
                obj.generateAllBussesAndHeaders();
            end

            contentH = fileread(hdrPath);
            if ~contains(contentH, '// --- PARAM PROTOTYPES START ---')
                obj.generateAllBussesAndHeaders();
                contentH = fileread(hdrPath);
            end

            contentS = fileread(srcPath);
            if ~contains(contentS, '// --- PARAM IMPLEMENTATIONS START ---')
                obj.generateAllBussesAndHeaders();
                contentS = fileread(srcPath);
            end

            newContentH = regexprep(contentH, ...
                '(?s)// --- PARAM PROTOTYPES START ---.*?// --- PARAM PROTOTYPES END ---', ...
                sprintf('// --- PARAM PROTOTYPES START ---\n%s// --- PARAM PROTOTYPES END ---', protoStr));

            if ~strcmp(contentH, newContentH)
                fid = fopen(hdrPath, 'w');
                fprintf(fid, '%s', newContentH);
                fclose(fid);
            end

            newContentS = regexprep(contentS, ...
                '(?s)// --- PARAM IMPLEMENTATIONS START ---.*?// --- PARAM IMPLEMENTATIONS END ---', ...
                sprintf('// --- PARAM IMPLEMENTATIONS START ---\n%s// --- PARAM IMPLEMENTATIONS END ---', implStr));

            if ~strcmp(contentS, newContentS)
                fid = fopen(srcPath, 'w');
                fprintf(fid, '%s', newContentS);
                fclose(fid);
            end
        end

        function binding = getParamBindingFromBlock(obj, block, direction)
            % GETPARAMBINDINGFROMBLOCK - Convert one masked block to manifest data.
            binding = [];
            try
                if strcmp(get_param(block, 'Commented'), 'on')
                    return;
                end
                pName = strtrim(char(string(get_param(block, 'param_name'))));
                pType = lower(strtrim(char(string(get_param(block, 'param_type')))));
            catch
                return; % Not a parameter mask.
            end

            if isempty(pName) || strcmp(pName, '<empty>')
                return;
            end
            px4Name = upper(pName);
            if isempty(regexp(px4Name, '^[A-Z][A-Z0-9_]*$', 'once'))
                error('[px4API:InvalidParameterName] "%s" at %s is not a valid PX4 parameter name.', ...
                    pName, obj.blockToPath(block));
            end

            if any(strcmp(pType, {'single', 'float', 'float32'}))
                typeName = 'float';
            elseif any(strcmp(pType, {'int32', 'int32_t'}))
                typeName = 'int32';
            else
                error('[px4API:InvalidParameterType] "%s" at %s must be float/single or int32.', ...
                    pType, obj.blockToPath(block));
            end

            direction = lower(char(string(direction)));
            if isempty(direction)
                direction = obj.inferParamDirection(block);
            end
            if ~any(strcmp(direction, {'read', 'write'}))
                return;
            end

            binding = struct('px4_name', px4Name, ...
                'symbol', [lower(px4Name), '_', typeName], ...
                'type', typeName, ...
                'read', strcmp(direction, 'read'), ...
                'write', strcmp(direction, 'write'), ...
                'blocks', {{obj.blockToPath(block)}});
        end

        function bindings = mergeParamBinding(obj, bindings, candidate)
            % One physical PX4 parameter has exactly one declared type/handle.
            for i = 1:numel(bindings)
                if strcmp(bindings(i).px4_name, candidate.px4_name)
                    if ~strcmp(bindings(i).type, candidate.type)
                        error('[px4API:ParameterTypeConflict] PX4 parameter %s is configured as both %s and %s (including %s).', ...
                            candidate.px4_name, bindings(i).type, candidate.type, candidate.blocks{1});
                    end
                    bindings(i).read = bindings(i).read || candidate.read;
                    bindings(i).write = bindings(i).write || candidate.write;
                    if ~any(strcmp(bindings(i).blocks, candidate.blocks{1}))
                        bindings(i).blocks{end+1} = candidate.blocks{1};
                    end
                    return;
                end
            end
            bindings(end+1) = candidate;
        end

        function bindings = removeBlockFromParamBindings(~, bindings, blockPath)
            % REMOVEBLOCKFROMPARAMBINDINGS - Prune one block's stale registration.
            % Keep an entry if one or more other blocks still refer to it.
            keep = true(1, numel(bindings));
            for i = 1:numel(bindings)
                blockList = bindings(i).blocks;
                if ischar(blockList) || isstring(blockList)
                    blockList = cellstr(blockList);
                end
                blockList = blockList(~strcmp(blockList, blockPath));
                if isempty(blockList)
                    keep(i) = false;
                else
                    bindings(i).blocks = blockList;
                end
            end
            bindings = bindings(keep);
        end

        function direction = inferParamDirection(~, block)
            direction = '';
            try
                maskType = lower(char(string(get_param(block, 'MaskType'))));
                reference = lower(char(string(get_param(block, 'ReferenceBlock'))));
                identity = [maskType, ' ', reference];
                if contains(identity, 'param_read') || contains(identity, 'parameter read')
                    direction = 'read'; return;
                elseif contains(identity, 'param_write') || contains(identity, 'parameter write')
                    direction = 'write'; return;
                end
                % Compatibility fallback for existing library blocks.
                ports = get_param(block, 'Ports');
                if ports(2) > 0
                    direction = 'read';
                elseif ports(1) > 0
                    direction = 'write';
                end
            catch
            end
        end

        function setParamNameConstant(~, constantPath, paramName)
            % SETPARAMNAMECONSTANT - Mirror a mask name into hidden uint8[17] data.
            px4Name = upper(strtrim(char(string(paramName))));
            bytes = uint8(px4Name);
            if numel(bytes) > 16
                error('[px4API:ParameterNameTooLong] PX4 parameter "%s" exceeds the 16-character limit.', px4Name);
            end
            data = zeros(1, 17, 'uint8');
            data(1:numel(bytes)) = bytes;
            set_param(constantPath, 'Value', mat2str(double(data)));
            set_param(constantPath, 'OutDataTypeStr', 'uint8');
        end

        function path = blockToPath(~, block)
            try
                path = getfullname(block);
            catch
                path = char(string(block));
            end
        end

        function [needed, reason] = needsGeneration(obj)
            % NEEDSGENERATION - Timestamp-based dependency checker.
            %
            % Compares the modification times of PX4 .msg files and generator 
            % MATLAB scripts against the generated artifacts. Triggers a clean 
            % rebuild if any source dependency is newer than the generated files.
            %
            % Outputs:
            %   needed - Boolean flag indicating if regeneration is required
            %   reason - String describing the trigger condition
            
            needed = true;
            obj.LocalGeneratedDir = fullfile(obj.PackageRoot, 'generated_code');
            obj.EnumsDir = fullfile(obj.PackageRoot, '+enums');

            if ~exist(obj.LocalGeneratedDir, 'dir') || ~exist(obj.EnumsDir, 'dir')
                reason = 'mandatory package directories are missing';
                if ~exist(obj.LocalGeneratedDir, 'dir'), mkdir(obj.LocalGeneratedDir); end
                if ~exist(obj.EnumsDir, 'dir'), mkdir(obj.EnumsDir); end
                if isempty(strfind(path(), obj.LocalGeneratedDir)), addpath(obj.LocalGeneratedDir); end
                return; 
            end
            if isempty(strfind(path(), obj.LocalGeneratedDir)), addpath(obj.LocalGeneratedDir); end

            hdrPath = fullfile(obj.LocalGeneratedDir, obj.StubHeaderName);
            srcPath = fullfile(obj.LocalGeneratedDir, obj.StubSrcName);
            checkPath = fullfile(obj.LocalGeneratedDir, obj.OrbCacheFile);

            if ~(exist(hdrPath, 'file') == 2 && exist(srcPath, 'file') == 2 && exist(checkPath, 'file') == 2)
                reason = 'one or more generated source files are missing';
                obj.clearFolderContents(obj.LocalGeneratedDir, {obj.ParamBindingsFile});
                obj.clearFolderContents(obj.EnumsDir);
                return;
            end

            msgDir = fullfile(obj.PX4Root, 'msg');
            if ~exist(msgDir, 'dir')
                reason = 'PX4 msg directory is missing'; 
                return; 
            end

            msgFiles = dir(fullfile(msgDir, '**', '*.msg'));
            if isempty(msgFiles)
                needed = false; 
                reason = 'no PX4 .msg files were found'; 
                return; 
            end
            
            % Exclude legacy/backup message folders
            msgFiles(contains({msgFiles.folder}, [filesep 'px4_msgs_old'])) = [];
            newestMsg = max([msgFiles(:).datenum]);

            hdrTime = dir(hdrPath).datenum; 
            srcTime = dir(srcPath).datenum; 
            glueTime = dir(checkPath).datenum;

            generatorFiles = {fullfile(obj.PackageRoot, 'px4API.m'), fullfile(obj.PackageRoot, 'uORB_read.m'), ...
                              fullfile(obj.PackageRoot, 'uORB_write.m'), fullfile(obj.PackageRoot, 'uORB_msg.m'), fullfile(obj.PackageRoot, 'deployment.m')};
            genTimes = [];
            for i = 1:numel(generatorFiles)
                if exist(generatorFiles{i}, 'file') == 2
                    genTimes(end+1) = dir(generatorFiles{i}).datenum; 
                end
            end
            newestGenTime = max([genTimes, -inf]);

            if min([hdrTime, srcTime, glueTime]) >= max(newestMsg, newestGenTime)
                needed = false; 
                reason = 'generated files are newer';
            else
                reason = 'dependencies are newer';
                obj.clearFolderContents(obj.LocalGeneratedDir, {obj.ParamBindingsFile});
                obj.clearFolderContents(obj.EnumsDir);
            end
        end

        function clearFolderContents(~, folderPath, preservedFiles)
            % CLEARFOLDERCONTENTS - Safely purges files inside a directory.
            %
            % Deletes files and subdirectories without deleting the parent folder 
            % itself. This prevents MATLAB from throwing path corruption warnings 
            % when active directories are forcefully removed.
            
            if nargin < 3
                preservedFiles = {};
            end
            if exist(folderPath, 'dir') == 7
                items = dir(folderPath);
                for i = 1:length(items)
                    if strcmp(items(i).name, '.') || strcmp(items(i).name, '..')
                        continue; 
                    end
                    if ~items(i).isdir && any(strcmp(items(i).name, preservedFiles))
                        continue;
                    end
                    fullItemPath = fullfile(items(i).folder, items(i).name);
                    if items(i).isdir
                        rmdir(fullItemPath, 's'); 
                    else 
                        delete(fullItemPath); 
                    end
                end
            end
        end

        function fieldMetadata = getFieldMetadataFromCache(obj, topicName)
            % GETFIELDMETADATAFROMCACHE - Retrieves struct field layout from the JSON cache.
            %
            % Used by the glue code generator to identify float32/float64 fields 
            % so they can be initialized to NaN during message allocation.
            %
            % Inputs:
            %   topicName - snake_case uORB topic name
            %
            % Outputs:
            %   fieldMetadata - MATLAB table containing fieldName, fieldType, and arraySize
            
            fieldMetadata = table();
            if isfield(obj.OrbCache, 'topics') && isfield(obj.OrbCache.topics, topicName)
                topicEntry = obj.OrbCache.topics.(topicName);
                if isfield(topicEntry, 'fields')
                    fieldsArray = topicEntry.fields;
                    while iscell(fieldsArray) && isscalar(fieldsArray)
                        fieldsArray = fieldsArray{1};
                    end
                    if iscell(fieldsArray), fieldsArray = [fieldsArray{:}]; end
                    if isstruct(fieldsArray) && ~isempty(fieldsArray)
                        fieldMetadata = table({fieldsArray(:).name}', {fieldsArray(:).type}', ...
                            [fieldsArray(:).arraySize]', 'VariableNames', {'fieldName', 'fieldType', 'arraySize'});
                    end
                end
            end
        end

        function topic = resolveOrbTopic(obj, name)
            % RESOLVEORBTOPIC Keep explicit variants; map base aliases to the first topic.
            base = obj.getBaseTopicForVariant(name);
            variants = obj.getTopicVariants(base);
            topic = name;
            if ~any(strcmp(variants, name)), topic = variants{1}; end
        end

        function variants = getTopicVariants(obj, topicName)
            % GETTOPICVARIANTS - Resolves uORB topic variants from the cache.
            %
            % Normalize cached char/string/cell representations into a row cell
            % array so consumers can concatenate topic names consistently.
            %
            % Inputs:
            %   topicName - Base or variant topic name
            %
            % Outputs:
            %   variants - Cell array of strings representing all valid ORB_IDs
            
            variants = {topicName}; % Default fallback
            if ~isfield(obj.OrbCache, 'topics')
                return; 
            end

            if isfield(obj.OrbCache.topics, topicName)
                entry = obj.OrbCache.topics.(topicName);
                if isfield(entry, 'variants') && ~isempty(entry.variants)
                    variants = entry.variants;
                end
            else
                topicKeys = fieldnames(obj.OrbCache.topics);
                for i = 1:length(topicKeys)
                    baseName = topicKeys{i};
                    entry = obj.OrbCache.topics.(baseName);
                    if isfield(entry, 'variants')
                        v = entry.variants;
                        if (ischar(v) && strcmp(v, topicName)) || (iscell(v) && any(strcmp(v, topicName)))
                            variants = v; 
                            break;
                        end
                    end
                end
            end
            
            % Enforce cell array output type
            if ischar(variants) || isstring(variants)
                variants = {variants};
            elseif iscell(variants)
                variants = variants(~cellfun(@isempty, variants));
                if isempty(variants)
                    variants = {topicName}; 
                end
            end
            variants = reshape(variants, 1, []);
        end

        function baseTopic = getBaseTopicForVariant(obj, topicName)
            % GETBASETOPICFORVARIANT - Maps a variant name back to its base message struct.
            %
            % For example, vehicle_attitude_groundtruth uses vehicle_attitude_s.
            % Cached variants may be stored as char vectors or cell arrays.
            
            baseTopic = topicName;
            if isfield(obj.OrbCache, 'topics')
                if isfield(obj.OrbCache.topics, topicName)
                    return; 
                end
                for key = fieldnames(obj.OrbCache.topics)'
                    entry = obj.OrbCache.topics.(key{1});
                    if isfield(entry, 'variants')
                        v = entry.variants;
                        if (ischar(v) && strcmp(v, topicName)) || (iscell(v) && any(strcmp(v, topicName)))
                            baseTopic = key{1}; 
                            return;
                        end
                    end
                end
            end
        end

        function saveOrbCache(obj)
            % SAVEORBCACHE - Persists the in-memory metadata cache to a JSON file.
            %
            % Stores parsed fields and variants so a new API instance can
            % restore buses without parsing every .msg file again.
            
            if ~exist(obj.LocalGeneratedDir, 'dir')
                mkdir(obj.LocalGeneratedDir); 
            end
            cachePath = fullfile(obj.LocalGeneratedDir, obj.OrbCacheFile);
            if ~isfield(obj.OrbCache, 'timestamp')
                obj.OrbCache.timestamp = datetime('now'); 
            end
            if ~isfield(obj.OrbCache, 'topics')
                obj.OrbCache.topics = struct(); 
            end
            fid = fopen(cachePath, 'w');
            if fid ~= -1
                fprintf(fid, '%s', jsonencode(obj.OrbCache, 'PrettyPrint', true)); 
                fclose(fid); 
            end
        end

        function loadOrbCacheFromJson(obj)
            % LOADORBCACHEFROMJSON - Restores the metadata cache from the JSON file.
            
            cachePath = fullfile(obj.LocalGeneratedDir, obj.OrbCacheFile);
            if ~exist(cachePath, 'file')
                return; 
            end
            obj.OrbCache = jsondecode(fileread(cachePath));
            if ~isfield(obj.OrbCache, 'topics')
                obj.OrbCache.topics = struct(); 
            end
            obj.OrbCacheLoaded = true;
        end

        function generateAllBussesAndHeaders(obj)
            % GENERATEALLBUSSESANDHEADERS - The Monolithic Static API Generator.
            %
            % Scans all PX4 .msg files, topologically sorts struct dependencies, 
            % and generates a complete, static C header and source file. 
            %
            % Generate the complete API for C Caller function discovery. Blocks
            % share this header; selecting a topic does not rewrite it. Hardware
            % glue is generated separately for calls used by the current build.
            
            msgDir = fullfile(obj.PX4Root, 'msg');
            if ~exist(msgDir, 'dir')
                error('[px4API:Error] Missing PX4 msg directory at: %s', msgDir);
            end

            % Discover all .msg files and exclude legacy backups
            rawMsgFiles = dir(fullfile(msgDir, '**', '*.msg'));
            rawMsgFiles(contains({rawMsgFiles.folder}, [filesep 'px4_msgs_old'])) = [];

            if isempty(rawMsgFiles)
                msgFiles = rawMsgFiles;
            else
                % Sort files so that 'versioned' subfolders are processed LAST.
                % This ensures that unique(..., 'last') keeps the versioned file 
                % over the non-versioned file if both exist for the same topic.
                isVersioned = contains({rawMsgFiles.folder}, 'versioned');
                [~, sortIdx] = sort(isVersioned);
                rawMsgFiles = rawMsgFiles(sortIdx);

                % Deduplicate by topic name, keeping the last occurrence (versioned)
                topicNames = cell(1, length(rawMsgFiles));
                for idx = 1:length(rawMsgFiles)
                    [~, camelName, ~] = fileparts(rawMsgFiles(idx).name);
                    topicNames{idx} = obj.camelCaseToSnakeCase(camelName);
                end

                [~, uniqueIdx] = unique(topicNames, 'last');
                msgFiles = rawMsgFiles(uniqueIdx);
            end

            % Reset cache and tracking variables
            obj.OrbCache.topics = struct();
            allStructs = {};
            busAssignments = {};

            % Parse all messages, build dependency graph, and populate cache
            for i = 1:length(msgFiles)
                msgFilePath = fullfile(msgFiles(i).folder, msgFiles(i).name);
                [~, camelName, ~] = fileparts(msgFiles(i).name);
                topicName = obj.camelCaseToSnakeCase(camelName);
                
                [structString, busObj, deps, fieldMeta, topicVariants] = obj.generateBusFromMsg(camelName, topicName, msgFilePath);

                if ~isempty(structString)
                    allStructs{end+1, 1} = topicName;
                    allStructs{end, 2} = structString;
                    allStructs{end, 3} = deps;

                    if ~isempty(busObj)
                        busAssignments{end+1, 1} = topicName;
                        busAssignments{end, 2} = busObj;
                    end

                    if isempty(topicVariants)
                        topicVariants = {topicName};
                    end

                    % Initialize as a scalar struct to prevent "Scalar structure required" 
                    % errors when assigning additional fields later. Wrapping topicVariants 
                    % in a cell array forces MATLAB to store it as a single field value 
                    % rather than expanding it into a struct array.
                    topicEntry = struct('variants', {topicVariants});

                    if ~isempty(fieldMeta) && height(fieldMeta) > 0
                        fieldsArray = {};
                        for fIdx = 1:height(fieldMeta)
                            fieldsArray{end+1} = struct('name', fieldMeta.fieldName{fIdx}, ...
                                'type', fieldMeta.fieldType{fIdx}, 'arraySize', fieldMeta.arraySize(fIdx));
                        end
                        topicEntry.fields = {fieldsArray};
                    end

                    obj.OrbCache.topics.(topicName) = topicEntry;
                end
            end

            % Topological sort for local structs to prevent incomplete type errors
            if ~isempty(allStructs)
                allStructs = obj.topologicalSortStructs(allStructs);
            end

            % --- BUILD C HEADER ---
            h = sprintf('// Auto-generated by px4API\n#ifndef PX4_SIMULINK_API_H\n#define PX4_SIMULINK_API_H\n\n');
            h = sprintf('%s#include <stdint.h>\n#include <stdbool.h>\n\n', h);

            % PX4 Hardware/SITL: Use native uORB headers
            h = sprintf('%s#if defined(__PX4_NUTTX) || defined(__PX4_POSIX) || defined(__PX4_QURT) || defined(__PX4_CYGWIN)\n', h);
            for i = 1:size(allStructs, 1)
                h = sprintf('%s#include <uORB/topics/%s.h>\n', h, allStructs{i, 1});
            end
            h = sprintf('%s\n', h);
            for i = 1:size(allStructs, 1)
                h = sprintf('%stypedef struct %s_s %s_s;\n', h, allStructs{i, 1}, allStructs{i, 1});
            end
            
            % Local Desktop Simulation: Use generated mockup structs
            h = sprintf('%s#else\n', h);
            for i = 1:size(allStructs, 1)
                h = sprintf('%s%s\n', h, allStructs{i, 2});
            end
            h = sprintf('%s#endif\n\n', h);

            % C-Linkage Prototypes
            h = sprintf('%s#ifdef __cplusplus\nextern "C" {\n#endif\n\n', h);
            baseTopics = fieldnames(obj.OrbCache.topics);
            for i = 1:numel(baseTopics)
                base = baseTopics{i};
                variants = unique([{base}, obj.getTopicVariants(base)], 'stable');
                for j = 1:numel(variants)
                    topic = variants{j};
                    h = sprintf('%s%s_s read_%s(void);\n', h, base, topic);
                    h = sprintf('%svoid write_%s(%s_s in);\n', h, topic, base);
                    h = sprintf('%s%s_s init_%s(bool initialize_to_nan);\n', h, base, topic);
                end
            end
            h = sprintf('%suint64_t read_px4_system_time(void);\n\n', h);
            h = sprintf('%s// --- PARAM PROTOTYPES START ---\n', h);
            h = sprintf('%s// --- PARAM PROTOTYPES END ---\n\n', h);
            h = sprintf('%s#ifdef __cplusplus\n}\n#endif\n\n#endif // PX4_SIMULINK_API_H\n', h);

            % --- BUILD C SOURCE (Desktop stubs only) ---
            % Note: NO #if guards here. Simulink's parser gets confused by them.
            % This file is excluded from PX4 builds anyway via copyGeneratedCodeFiles.
            s = sprintf('#include "%s"\n#include <math.h>\n#include <string.h>\n\n', obj.StubHeaderName);
            s = sprintf('%suint64_t read_px4_system_time(void) { return 0; }\n\n', s);
            
            for i = 1:numel(baseTopics)
                base = baseTopics{i};
                variants = unique([{base}, obj.getTopicVariants(base)], 'stable');
                for j = 1:numel(variants)
                    topic = variants{j};
                    s = sprintf('%s%s_s read_%s(void) { %s_s empty = {0}; return empty; }\n', s, base, topic, base);
                    s = sprintf('%svoid write_%s(%s_s in) { (void)in; }\n', s, topic, base);
                    s = [s px4io.deployment.initializer(base, topic, obj.getFieldMetadataFromCache(base))];
                end
            end
            s = sprintf('%s// --- PARAM IMPLEMENTATIONS START ---\n', s);
            s = sprintf('%s// --- PARAM IMPLEMENTATIONS END ---\n', s);

            % Write generated files to disk
            if ~exist(obj.LocalGeneratedDir, 'dir')
                mkdir(obj.LocalGeneratedDir);
            end

            fid = fopen(fullfile(obj.LocalGeneratedDir, obj.StubHeaderName), 'w');
            fprintf(fid, '%s', h);
            fclose(fid);

            fid = fopen(fullfile(obj.LocalGeneratedDir, obj.StubSrcName), 'w');
            fprintf(fid, '%s', s);
            fclose(fid);

            % Restore the fixed generic parameter declarations after rebuilding the
            % monolithic uORB header/source pair.
            obj.writeParamArtifacts();
            
            % Assign buses to workspace (including all variants to guarantee scope parity)
            for i = 1:size(busAssignments, 1)
                baseName = busAssignments{i, 1};
                busObj = busAssignments{i, 2};
                assignin('base', [baseName, '_s'], busObj);
                
                if isfield(obj.OrbCache, 'topics') && isfield(obj.OrbCache.topics, baseName)
                    entry = obj.OrbCache.topics.(baseName);
                    if isfield(entry, 'variants')
                        variants = entry.variants;

                        if ischar(variants) || isstring(variants)
                            variants = {variants};
                        elseif iscell(variants)
                            variants = variants(~cellfun(@isempty, variants));
                        else
                            variants = {variants};
                        end

                        for v = 1:length(variants)
                            vName = variants{v};
                            if (ischar(vName) || isstring(vName)) && ~strcmp(vName, baseName)
                                assignin('base', [vName, '_s'], busObj);
                            end
                        end
                    end
                end
            end

            % Persist the metadata cache to disk for fast subsequent startups
            obj.saveOrbCache();

            if obj.ShowDebug
                fprintf('✓ Full static API generated.\n');
            end
        end

        function typeSize = getPx4FieldTypeSize(~, px4Type)
            % GETPX4FIELDTYPESIZE - Return the scalar field-size sorting key.
            % Built-in fields sort by decreasing size; nested types return zero.
            % This does not compute a complete structure size or insert padding.
            % PX4 builds use native uORB headers for the actual target layout.
            
            switch lower(px4Type)
                case {'uint64','int64','float64'}, typeSize = 8;
                case {'uint32','int32','float32'}, typeSize = 4;
                case {'uint16','int16'}, typeSize = 2;
                case {'uint8','int8','bool','char'}, typeSize = 1;
                otherwise, typeSize = 0;
            end
        end

        function [structStr, busObj, dependencies, fieldMetadata, topicVariants] = generateBusFromMsg(obj, camelName, topicName, msgFilePath)
            % GENERATEBUSFROMMSG - Parses a PX4 .msg file into C structs and Simulink Buses.
            %
            % Handles array extraction, nested struct dependency tracking, constant 
            % enumeration generation, and native uORB memory alignment sorting.
            %
            % Inputs:
            %   camelName  - Original CamelCase filename
            %   topicName  - snake_case uORB topic name
            %   msgFilePath - Absolute path to the .msg file
            %
            % Outputs:
            %   structStr    - C struct definition string
            %   busObj       - Simulink.Bus object
            %   dependencies - Cell array of required nested struct names
            %   fieldMetadata - Table of field properties for JSON caching
            %   topicVariants - Cell array of ORB_ID variants defined in the message
            
            if nargin == 2
                topicName = obj.camelCaseToSnakeCase(camelName); 
                msgFilePath = ''; 
            end
            
            structStr = ''; 
            busObj = []; 
            dependencies = {}; 
            fieldMetadata = table(); 
            topicVariants = {};
            
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
            lines = textscan(fid, '%s', 'Delimiter', '\n'); 
            fclose(fid); 
            lines = lines{1};

            structBody = sprintf('struct %s_s {\n', topicName);
            parsedFields = {}; 
            constantsList = {}; 
            fieldNames = {}; 
            fieldTypes = {}; 
            arraySizes = [];

            for i = 1:length(lines)
                line = strtrim(lines{i});
                if startsWith(line, '#')
                    stripped = regexprep(line, '^#\s*', ''); 
                    toks = strsplit(strtrim(stripped));
                    if ~isempty(toks) && strcmpi(toks{1}, 'TOPICS')
                        topicVariants = [topicVariants, toks(2:end)];
                    end
                    continue;
                end
                
                commentIdx = strfind(line, '#');
                if ~isempty(commentIdx)
                    line = strtrim(line(1:commentIdx(1)-1)); 
                end
                
                if contains(line, '=')
                    tokens = strsplit(line, '=');
                    if length(tokens) >= 2
                        typeAndName = strsplit(strtrim(tokens{1}));
                        if length(typeAndName) >= 2
                            constVal = strtrim(tokens{2}); 
                            if endsWith(constVal, ';')
                                constVal = constVal(1:end-1); 
                            end
                            constantsList{end+1, 1} = typeAndName{2}; 
                            constantsList{end, 2} = constVal;
                        end
                    end
                    continue; 
                end
                
                tokens = strsplit(line);
                if length(tokens) < 2
                    continue; 
                end
                
                px4Type = tokens{1}; 
                varName = tokens{2};
                if endsWith(varName, ';')
                    varName = varName(1:end-1); 
                end
                
                arraySize = 1;
                arrayMatch = regexp([px4Type, ' ', varName], '\[(\d+)\]', 'tokens');
                if ~isempty(arrayMatch)
                    arraySize = str2double(arrayMatch{1}{1}); 
                end
                
                px4Type = regexprep(px4Type, '\[\d+\]', ''); 
                varName = regexprep(varName, '\[\d+\]', '');
                
                if startsWith(varName, 'sl_padding')
                    continue; 
                end
                
                [cType, slType, isDependency, depName] = obj.px4TypeToCTypesWithDeps(px4Type);
                if isDependency
                    depNameSnake = obj.camelCaseToSnakeCase(depName);
                    if ~any(strcmp(dependencies, depNameSnake))
                        dependencies{end+1} = depNameSnake; 
                    end
                end
                
                parsedFields{end+1} = struct('px4Type', px4Type, 'varName', varName, 'arraySize', arraySize, ...
                    'cType', cType, 'slType', slType, 'isDependency', isDependency, 'depName', depName);
            end

            % Reorder fields by size (descending) to match PX4 uORB memory layout
            if ~isempty(parsedFields)
                fieldSizes = cellfun(@(f) obj.getPx4FieldTypeSize(f.px4Type), parsedFields);
                [~, order] = sort(fieldSizes, 'descend'); 
                parsedFields = parsedFields(order);
                parsedFieldsStruct = [parsedFields{:}];
                fieldNames = {parsedFieldsStruct.varName}; 
                fieldTypes = {parsedFieldsStruct.px4Type}; 
                arraySizes = [parsedFieldsStruct.arraySize];
            end

            for i = 1:numel(parsedFields)
                f = parsedFields{i};
                if f.arraySize > 1
                    structBody = sprintf('%s    %s %s[%d];\n', structBody, f.cType, f.varName, f.arraySize);
                else
                    structBody = sprintf('%s    %s %s;\n', structBody, f.cType, f.varName); 
                end
            end

            if ~isempty(topicVariants)
                topicVariants = unique(topicVariants, 'stable'); 
            end
            
            if ~isempty(constantsList)
                enumFileName = fullfile(obj.PackageRoot, '+enums', [topicName '.m']);
                efid = fopen(enumFileName, 'w');
                if efid ~= -1
                    fprintf(efid, 'classdef %s < Simulink.IntEnumType\n    enumeration\n', topicName);
                    for cIdx = 1:size(constantsList, 1)
                        fprintf(efid, '        %s(%s)\n', constantsList{cIdx, 1}, constantsList{cIdx, 2}); 
                    end
                    fprintf(efid, '    end\nend\n'); 
                    fclose(efid); 
                    clear(topicName);
                end
            end

            if ~isempty(parsedFields)
                nativeStructName = [lower(topicName), '_s'];
                structStr = sprintf('%s};\ntypedef struct %s %s;\n', structBody, nativeStructName, nativeStructName);
                busObj = obj.createBusFromFieldData(parsedFields);
                fieldMetadata = table(fieldNames', fieldTypes', arraySizes', 'VariableNames', {'fieldName', 'fieldType', 'arraySize'});
            end
        end

        function exportGeneratedCode(obj, buildInfo)
            % EXPORTGENERATEDCODE - Post-code generation Simulink callback hook.
            %
            % Stages a complete deployment, generates the model-agnostic wrapper
            % and the hardware-specific C++ glue code, and copies all necessary 
            % artifacts into the PX4 firmware tree.
            
            fprintf('\n--- [px4API] Preparing generated deployment ---\n');
            if isempty(obj.AllowedExtensions)
                return; 
            end
            if ~exist(obj.PX4Root, 'dir')
                error('[px4API:Error] Missing PX4 Root'); 
            end
            obj.ResolvedExternalDir = fullfile(obj.resolveAbsolutePath(obj.PX4Root), 'src', 'modules', obj.PX4ModuleName, 'generated_code');
            parent = fileparts(obj.ResolvedExternalDir);
            if ~isfolder(parent), error('px4io:MissingModule', 'Module directory does not exist: %s', parent); end
            stage = tempname(parent); mkdir(stage);
            cleanup = onCleanup(@() px4io.deployment.cleanupDirectory(stage));
            if ~exist(obj.LocalGeneratedDir, 'dir')
                obj.generateAllBussesAndHeaders(); 
            end

            modelName = buildInfo.getBuildName;
            % Reconcile bindings from the model for the generated PX4 bridge.
            % The parameter API signatures stay fixed; only the binding table
            % changes when blocks are added, deleted, or renamed.
            obj.syncParamBindings(modelName);
            obj.generateModelWrapper(modelName, stage);
            obj.generateOmnipotentCppGlue(stage, buildInfo);
            
            sources = px4io.deployment.sourceFiles(buildInfo);
            includes = buildInfo.getFullFileList('include');
            buildDirInfo = RTW.getBuildDir(modelName);
            headers = dir(fullfile(buildDirInfo.BuildDirectory, '*.h'));
            headerFiles = arrayfun(@(f) fullfile(f.folder, f.name), headers, 'UniformOutput', false);
            files = unique([sources(:); includes(:); headerFiles(:)], 'stable');
            count = 0;
            for i = 1:numel(files)
                [~, name, ext] = fileparts(files{i});
                if strcmp([name ext], obj.StubHeaderName), continue; end
                if ismember(lower(ext), obj.AllowedExtensions)
                    if ~isfile(files{i}), error('px4io:MissingFile', 'Missing build file: %s', files{i}); end
                    px4io.deployment.copyFile(files{i}, fullfile(stage, [name ext]));
                    count = count + 1;
                end
            end
            copyfile(fullfile(obj.LocalGeneratedDir, obj.StubHeaderName), fullfile(stage, obj.StubHeaderName));
            px4io.deployment.replaceDirectory(stage, obj.ResolvedExternalDir);

            fprintf('Successfully moved %d generated files over to PX4 code tree.\n', count);
        end

        function count = copyGeneratedCodeFiles(obj, sourceDir, destDir, recursive)
            % COPYGENERATEDCODEFILES - Recursively copies filtered files to the PX4 tree.
            %
            % This general copy helper excludes desktop stubs. The deployment
            % path uses BuildInfo and deployment.copyFile instead, which also
            % validate source selection and detect conflicting filenames.
            
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
                    continue; 
                end
                
                sourcePath = fullfile(entries(i).folder, entries(i).name);
                destPath = fullfile(destDir, entries(i).name);
                
                if entries(i).isdir
                    if recursive
                        count = count + obj.copyGeneratedCodeFiles(sourcePath, destPath, true); 
                    end
                else
                    [~, ~, ext] = fileparts(entries(i).name);
                    if strcmp(entries(i).name, obj.StubSrcName)
                        continue; % Skip desktop stubs
                    end
                    if ismember(lower(ext), extAllowed)
                        copyfile(sourcePath, destPath, 'f'); 
                        count = count + 1; 
                    end
                end
            end
        end

        function generateOmnipotentCppGlue(obj, outputDir, buildInfo)
            % GENERATEOMNIPOTENTCPPGLUE - Generates the PX4 hardware uORB/Param router.
            %
            % Scan BuildInfo sources for message and clock calls, then filter
            % the discovered names against cached PX4 topics. Parameter bindings
            % come from the model manifest, not from the source scan.
            % Emit native uORB subscriptions/publications, message initializers,
            % parameter caches, and bridge initialization/termination functions.
            % Source scanning is lexical and does not evaluate C preprocessor paths.
            
            if nargin < 2 || isempty(outputDir)
                outputDir = obj.ResolvedExternalDir; 
            end
            
            files = px4io.deployment.sourceFiles(buildInfo);
            [readTopics, writeTopics, initTopics, hasSystemTime] = px4io.deployment.discoverCalls(files);
            knownTopics = fieldnames(obj.OrbCache.topics)';
            bases = knownTopics;
            for i = 1:numel(bases)
                knownTopics = [knownTopics obj.getTopicVariants(bases{i})];
            end
            readTopics = intersect(readTopics, knownTopics);
            writeTopics = intersect(writeTopics, knownTopics);
            initTopics = intersect(initTopics, knownTopics);

            % Use the physical PX4 names and types recorded by parameter blocks;
            % generated C function names do not identify individual parameters.
            paramBindings = obj.loadParamBindings();

            % Build C++ Source String
            cppStr = sprintf('// Auto-generated selective strongly-typed return-by-value uORB routing layer\n');
            cppStr = sprintf('%s#include <px4_platform_common/defines.h>\n#include <px4_platform_common/log.h>\n', cppStr);
            cppStr = sprintf('%s#include <uORB/uORB.h>\n#include <uORB/Publication.hpp>\n#include <uORB/Subscription.hpp>\n', cppStr);
            cppStr = sprintf('%s#include <string.h>\n#include <math.h>\n#include <parameters/param.h>\n#include <uORB/topics/parameter_update.h>\n\n', cppStr);
            cppStr = sprintf('%s#include "%s"\n\n', cppStr, obj.StubHeaderName);
            if hasSystemTime
                cppStr = sprintf('%s#include <drivers/drv_hrt.h>\n', cppStr); 
            end

            allTopics = unique([readTopics, writeTopics, initTopics]);
            for i = 1:length(allTopics)
                cppStr = sprintf('%s#include <uORB/topics/%s.h>\n', cppStr, obj.getBaseTopicForVariant(allTopics{i}));
            end

            % C++ Singleton Class for uORB Handles
            cppStr = sprintf('%s\nclass SimulinkGlue {\npublic:\n\tSimulinkGlue() {}\n\t~SimulinkGlue() {}\n\n', cppStr);
            for i = 1:length(writeTopics)
                t = writeTopics{i}; 
                b = obj.getBaseTopicForVariant(t);
                cppStr = sprintf('%s\tuORB::Publication<%s_s> _%s_pub{ORB_ID(%s)};\n', cppStr, b, t, obj.resolveOrbTopic(t));
            end
            for i = 1:length(readTopics)
                t = readTopics{i}; 
                b = obj.getBaseTopicForVariant(t);
                cppStr = sprintf('%s\tuORB::Subscription _%s_sub{ORB_ID(%s)};\n', cppStr, t, obj.resolveOrbTopic(t));
                cppStr = sprintf('%s\t%s_s _%s_buffer{};\n', cppStr, b, t);
            end
            cppStr = sprintf('%s};\n\nstatic SimulinkGlue g_glue_instance;\nstatic bool g_initialized = false;\n\n', cppStr);
            cppStr = sprintf('%sextern "C" {\n\n', cppStr);

            if hasSystemTime
                cppStr = sprintf('%suint64_t read_px4_system_time(void) {\n\treturn hrt_absolute_time();\n}\n\n', cppStr);
            end

            % uORB Readers/Writers/Inits
            for i = 1:length(readTopics)
                t = readTopics{i}; 
                b = obj.getBaseTopicForVariant(t);
                cppStr = sprintf('%s%s_s read_%s(void) {\n', cppStr, b, t);
                cppStr = sprintf('%s\tif (g_initialized && g_glue_instance._%s_sub.updated()) g_glue_instance._%s_sub.copy(&g_glue_instance._%s_buffer);\n', cppStr, t, t, t);
                cppStr = sprintf('%s\treturn g_glue_instance._%s_buffer;\n}\n\n', cppStr, t);
            end
            for i = 1:length(writeTopics)
                t = writeTopics{i}; 
                b = obj.getBaseTopicForVariant(t);
                cppStr = sprintf('%svoid write_%s(%s_s in) {\n\tif (g_initialized) g_glue_instance._%s_pub.publish(in);\n}\n\n', cppStr, t, b, t);
            end
            for i = 1:numel(initTopics)
                topic = initTopics{i};
                base = obj.getBaseTopicForVariant(topic);
                cppStr = [cppStr px4io.deployment.initializer(base, topic, obj.getFieldMetadataFromCache(base))];
            end

            % Stable generic parameter bridge. The model supplies a padded name,
            % while this generated table resolves PX4 handles only during init.
            cppStr = sprintf('%s// --- Cached Generic Parameter Bridge ---\n', cppStr);
            cppStr = sprintf('%stypedef struct {\n    const char *name;\n    param_t handle;\n    param_type_t type;\n    union { float f; int32_t i; } value;\n} simulink_param_t;\n\n', cppStr);
            if ~isempty(paramBindings)
                cppStr = sprintf('%sstatic simulink_param_t g_simulink_params[] = {\n', cppStr);
                for i = 1:length(paramBindings)
                    p = paramBindings(i);
                    px4Type = 'PARAM_TYPE_FLOAT';
                    if strcmp(p.type, 'int32'), px4Type = 'PARAM_TYPE_INT32'; end
                    cppStr = sprintf('%s    {"%s", PARAM_INVALID, %s, {0}},\n', cppStr, p.px4_name, px4Type);
                end
                cppStr = sprintf('%s};\nstatic constexpr size_t g_simulink_param_count = %d;\n', cppStr, length(paramBindings));
                cppStr = sprintf('%sstatic uORB::Subscription g_param_update_sub{ORB_ID(parameter_update)};\n\n', cppStr);
                cppStr = sprintf('%sstatic int find_simulink_param(const uint8_t name[17], param_type_t type) {\n', cppStr);
                cppStr = sprintf('%s    for (size_t i = 0; i < g_simulink_param_count; ++i) {\n', cppStr);
                cppStr = sprintf('%s        if (g_simulink_params[i].type == type && strncmp(reinterpret_cast<const char *>(name), g_simulink_params[i].name, 17) == 0) return (int)i;\n', cppStr);
                cppStr = sprintf('%s    }\n    return -1;\n}\n\n', cppStr);
                cppStr = sprintf('%sstatic void init_simulink_params() {\n', cppStr);
                cppStr = sprintf('%s    for (size_t i = 0; i < g_simulink_param_count; ++i) {\n', cppStr);
                cppStr = sprintf('%s        g_simulink_params[i].value = {};\n        g_simulink_params[i].handle = param_find(g_simulink_params[i].name);\n', cppStr);
                cppStr = sprintf('%s        if (g_simulink_params[i].handle != PARAM_INVALID && param_type(g_simulink_params[i].handle) == g_simulink_params[i].type) {\n', cppStr);
                cppStr = sprintf('%s            if (g_simulink_params[i].type == PARAM_TYPE_FLOAT) (void)param_get(g_simulink_params[i].handle, &g_simulink_params[i].value.f);\n', cppStr);
                cppStr = sprintf('%s            else (void)param_get(g_simulink_params[i].handle, &g_simulink_params[i].value.i);\n', cppStr);
                cppStr = sprintf('%s        } else if (g_simulink_params[i].handle == PARAM_INVALID) {\n            PX4_WARN("parameter not found: %%s", g_simulink_params[i].name);\n            if (g_simulink_params[i].type == PARAM_TYPE_FLOAT) g_simulink_params[i].value.f = NAN;\n        } else {\n            PX4_ERR("parameter type mismatch: %%s", g_simulink_params[i].name);\n            g_simulink_params[i].handle = PARAM_INVALID;\n            if (g_simulink_params[i].type == PARAM_TYPE_FLOAT) g_simulink_params[i].value.f = NAN;\n        }\n', cppStr);
                cppStr = sprintf('%s    }\n}\n\n', cppStr);
                cppStr = sprintf('%sstatic void update_simulink_params_impl() {\n    if (!g_param_update_sub.updated()) return;\n    parameter_update_s update;\n    g_param_update_sub.copy(&update);\n', cppStr);
                cppStr = sprintf('%s    for (size_t i = 0; i < g_simulink_param_count; ++i) if (g_simulink_params[i].handle != PARAM_INVALID) {\n', cppStr);
                cppStr = sprintf('%s        if (g_simulink_params[i].type == PARAM_TYPE_FLOAT) (void)param_get(g_simulink_params[i].handle, &g_simulink_params[i].value.f);\n        else (void)param_get(g_simulink_params[i].handle, &g_simulink_params[i].value.i);\n    }\n}\n\n', cppStr);
            else
                cppStr = sprintf('%sstatic simulink_param_t g_simulink_params[1] = {};\nstatic int find_simulink_param(const uint8_t[17], param_type_t) { return -1; }\nstatic void init_simulink_params() {}\nstatic void update_simulink_params_impl() {}\n\n', cppStr);
            end
            cppStr = sprintf('%sfloat read_param_float(const uint8_t name[17]) { const int i = find_simulink_param(name, PARAM_TYPE_FLOAT); return i >= 0 ? g_simulink_params[i].value.f : NAN; }\n', cppStr);
            cppStr = sprintf('%sint32_t read_param_int32(const uint8_t name[17]) { const int i = find_simulink_param(name, PARAM_TYPE_INT32); return i >= 0 ? g_simulink_params[i].value.i : 0; }\n', cppStr);
            cppStr = sprintf('%svoid write_param_float(const uint8_t name[17], float value) { const int i = find_simulink_param(name, PARAM_TYPE_FLOAT); if (i >= 0 && g_simulink_params[i].handle != PARAM_INVALID && (!PX4_ISFINITE(g_simulink_params[i].value.f) || fabsf(g_simulink_params[i].value.f - value) > 1e-6f) && param_set(g_simulink_params[i].handle, &value) == PX4_OK) g_simulink_params[i].value.f = value; }\n', cppStr);
            cppStr = sprintf('%svoid write_param_int32(const uint8_t name[17], int32_t value) { const int i = find_simulink_param(name, PARAM_TYPE_INT32); if (i >= 0 && g_simulink_params[i].handle != PARAM_INVALID && g_simulink_params[i].value.i != value && param_set(g_simulink_params[i].handle, &value) == PX4_OK) g_simulink_params[i].value.i = value; }\n\n', cppStr);

            cppStr = sprintf('%svoid update_simulink_params(void) {\n', cppStr);
            cppStr = sprintf('%s    update_simulink_params_impl();\n', cppStr);
            cppStr = sprintf('%s}\n\n', cppStr);

            cppStr = sprintf('%svoid init_px4_simulink_io(void) {\n', cppStr);
            for i = 1:numel(readTopics)
                cppStr = sprintf('%s\tg_glue_instance._%s_buffer = {};\n', cppStr, readTopics{i});
            end
            cppStr = sprintf('%s\tg_initialized = true;\n', cppStr);
            cppStr = sprintf('%s\tinit_simulink_params();\n', cppStr);
            cppStr = sprintf('%s}\n\nvoid terminate_px4_simulink_io(void) {\n\tg_initialized = false;\n', cppStr);
            for i = 1:numel(readTopics)
                cppStr = sprintf('%s\tg_glue_instance._%s_sub.unsubscribe();\n', cppStr, readTopics{i});
            end
            if ~isempty(paramBindings)
                cppStr = sprintf('%s\tg_param_update_sub.unsubscribe();\n', cppStr);
            end
            cppStr = sprintf('%s}\n\n}\n', cppStr);

            if ~exist(outputDir, 'dir')
                mkdir(outputDir); 
            end
            fid = fopen(fullfile(outputDir, obj.GlueName), 'w');
            if fid == -1
                error('[px4API:Error] Could not write glue file'); 
            end
            fprintf(fid, '%s', cppStr); 
            fclose(fid);
        end
    end

    methods (Static)
        function api = getInstance()
            % GETINSTANCE - Reuse the initialized API during mask callbacks.
            % Restore missing workspace buses while keeping the persistent cache.
            % Reuse does not recheck file timestamps; forceGenerateAll explicitly
            % refreshes messages after PX4 interface changes in the same session.
            persistent sharedApi
            if isempty(sharedApi) || ~isvalid(sharedApi)
                sharedApi = px4io.px4API();
            end
            api = sharedApi;
            api.ensureBusesInWorkspace();
        end

        function forceGenerateAll()
            % FORCEGENERATEALL Reparse messages and replace API/bus/enum artifacts.
            % Use before model compilation after changing PX4 message definitions.
            px4api = px4io.px4API.getInstance();
            px4api.generateAllBussesAndHeaders();
        end   

        function resolvedPath = resolveAbsolutePath(pathStr)
            % RESOLVEABSOLUTEPATH - Normalizes paths (handles ~, ./, and ../).
            resolvedPath = pathStr;
            if startsWith(resolvedPath, '~/') || strcmp(resolvedPath, '~')
                resolvedPath = fullfile(getenv('HOME'), resolvedPath(2:end));
            elseif startsWith(resolvedPath, './') || startsWith(resolvedPath, '../')
                resolvedPath = char(java.io.File(resolvedPath).getCanonicalPath());
            end
        end

        function snakeName = camelCaseToSnakeCase(camelName)
            % CAMELCASETOSNAKECASE - Converts PX4 CamelCase filenames to snake_case.
            %
            % E.g., 'VehicleAttitude' -> 'vehicle_attitude'
            % Handles acronym numbers safely (e.g., 'Ekf2Timestamps' -> 'ekf2_timestamps')
            snakeName = lower(regexprep(camelName, '([a-z0-9])([A-Z])', '$1_$2'));
        end

        function variants = extractMsgTopicVariants(msgFilePath, defaultTopicName)
            % EXTRACTMSGTOPICVARIANTS - Parses #TOPICS metadata from a .msg file.
            variants = {defaultTopicName};
            if ~exist(msgFilePath, 'file')
                return; 
            end
            fid = fopen(msgFilePath, 'r');
            if fid == -1
                return; 
            end
            lines = textscan(fid, '%s', 'Delimiter', '\n'); 
            fclose(fid); 
            lines = lines{1};
            for i = 1:length(lines)
                line = strtrim(lines{i});
                if startsWith(line, '#')
                    toks = strsplit(strtrim(regexprep(line, '^#\s*', '')));
                    if ~isempty(toks) && strcmpi(toks{1}, 'TOPICS')
                        variants = [variants, toks(2:end)];
                    end
                end
            end
            variants = unique(variants(~cellfun(@isempty, variants)), 'stable');
            if isempty(variants)
                variants = {defaultTopicName}; 
            end
        end

        function [cType, slType, isDependency, depName] = px4TypeToCTypesWithDeps(px4Type)
            % PX4TYPETOCTYPESWITHDEPS - Maps PX4 primitives to C/Simulink types.
            %
            % Detects nested struct dependencies to build the topological sort graph.
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
                    isDependency = true; 
                    depName = px4Type;
                    structNameSnake = px4io.px4API.camelCaseToSnakeCase(px4Type);
                    slType = [structNameSnake, '_s']; 
                    cType = ['struct ', structNameSnake, '_s'];
            end
        end

        function orderedStructs = topologicalSortStructs(allStructs)
            % TOPOLOGICALSORTSTRUCTS - Sorts C structs via Depth-First Search.
            %
            % Ensures nested structs are defined before the structs that reference them,
            % preventing "incomplete type" C compiler errors.
            if isempty(allStructs)
                orderedStructs = {}; 
                return; 
            end
            numStructs = size(allStructs, 1);
            struct_map = containers.Map();
            for i = 1:numStructs
                struct_map(allStructs{i, 1}) = i; 
            end
            visited = false(numStructs, 1); 
            orderedStructs = {};
            for i = 1:numStructs
                if ~visited(i)
                    [orderedStructs, visited] = px4io.px4API.dfs_visit(i, allStructs, struct_map, visited, orderedStructs);
                end
            end
        end

        function [orderedStructs, visited] = dfs_visit(idx, allStructs, struct_map, visited, orderedStructs)
            % DFS_VISIT - Recursive helper for topological sorting.
            if visited(idx)
                return; 
            end
            visited(idx) = true;
            dependencies = allStructs{idx, 3};
            for i = 1:length(dependencies)
                depName = dependencies{i};
                if struct_map.isKey(depName)
                    depIdx = struct_map(depName);
                    if ~visited(depIdx)
                        [orderedStructs, visited] = px4io.px4API.dfs_visit(depIdx, allStructs, struct_map, visited, orderedStructs);
                    end
                end
            end
            orderedStructs{end+1, 1} = allStructs{idx, 1}; 
            orderedStructs{end, 2} = allStructs{idx, 2}; 
        end

        function listStr = getTopicDropdownString()
            % GETTOPICDROPDOWNSTRING - Generates comma-separated list of all uORB topics.
            %
            % Used by Simulink mask callbacks to populate dropdown menus.
            api = px4io.px4API.getInstance();
            if isfield(api.OrbCache, 'topics') && ~isempty(fieldnames(api.OrbCache.topics))
                topics = {};
                for key = fieldnames(api.OrbCache.topics)'
                    entry = api.OrbCache.topics.(key{1});
                    variants = entry.variants;
                    if ischar(variants) || isstring(variants)
                        variants = {char(variants)};
                    end
                    topics = [topics, variants]; %#ok<AGROW>
                end
                listStr = strjoin(unique(topics(~cellfun(@isempty, topics))), ',');
                return;
            end
            files = dir(fullfile(api.PX4Root, 'msg', '**', '*.msg'));
            topics = {};
            for i = 1:length(files)
                [~, camelName, ~] = fileparts(files(i).name);
                topicName = px4io.px4API.camelCaseToSnakeCase(camelName);
                if strcmp(topicName, 'message_version')
                    continue; 
                end
                variants = px4io.px4API.extractMsgTopicVariants(fullfile(files(i).folder, files(i).name), topicName);
                topics = [topics, variants];
            end
            listStr = strjoin(unique(topics(~cellfun(@isempty, topics))), ',');
        end

        function busObj = createBusFromFieldData(fieldsData)
            % CREATEBUSFROMFIELDDATA - Constructs a Simulink.Bus object from parsed metadata.
            %
            % Handles both initial generation (cell array of structs) and JSON cache 
            % loading (struct array) robustly.
            
            if isempty(fieldsData)
                busObj = []; 
                return; 
            end
            
            elements = [];
            isCellData = iscell(fieldsData);
            
            for fIdx = 1:length(fieldsData)
                if isCellData
                    f = fieldsData{fIdx};
                else
                    f = fieldsData(fIdx);
                end
                
                % Determine field name and Simulink data type
                if isfield(f, 'varName')
                    fName = f.varName;
                    fType = f.slType;
                elseif isfield(f, 'name')
                    fName = f.name;
                    [~, fType, ~, ~] = px4io.px4API.px4TypeToCTypesWithDeps(f.type);
                else
                    continue;
                end
                
                % Determine dimensions
                dim = 1;
                if isfield(f, 'arraySize')
                    dim = f.arraySize;
                    if iscell(dim)
                        dim = dim{1}; 
                    end
                    if isstring(dim) || ischar(dim)
                        dim = str2double(dim);
                    end
                end

                elem = Simulink.BusElement;
                elem.Name = fName;
                elem.DataType = fType;
                elem.Dimensions = dim;
                elem.Complexity = 'real';
                elem.SampleTime = -1;
                elem.DimensionsMode = 'Fixed';
                elements = [elements; elem];
            end
            
            if isempty(elements)
                busObj = []; 
                return;
            end

            busObj = Simulink.Bus;
            busObj.Elements = elements;
            busObj.DataScope = 'Imported';
            busObj.HeaderFile = 'px4_simulink_api.h';
        end

        function runPostCodeGen(buildInfo)
            % RUNPOSTCODEGEN - Export a completed Simulink build to PX4.
            % Does not change model settings, rerun code generation, or build PX4.
            % Message buses must already match the target before model compilation.
            apiInstance = px4io.px4API.getInstance();
            apiInstance.exportGeneratedCode(buildInfo);
        end

        function uorb_topic_callback(callbackContext)
            % UORB_TOPIC_CALLBACK - Mask callback to populate uORB topic dropdowns.
            blockHandle = callbackContext.BlockHandle;
            choices = strsplit(px4io.px4API.getTopicDropdownString(), ',');
            set_param(blockHandle, 'TypeOptions_uorb_topic', choices);
        end

        function generateModelWrapper(modelName, outputDir)
            % GENERATEMODELWRAPPER Delegate compiled-interface validation and emission.
            % Timing and entry-point names come from codeInfo.mat, not model-name guesses.
            px4io.deployment.writeWrapper(modelName, outputDir);
        end

        function listStr = getBaseTopicsDropdownString()
            % GETBASETOPICSDROPDOWNSTRING - Generates list of unique base message types.
            %
            % Used to populate the primary dropdown, excluding redundant variant names.
            api = px4io.px4API.getInstance();
            if isfield(api.OrbCache, 'topics') && ~isempty(fieldnames(api.OrbCache.topics))
                baseTopics = fieldnames(api.OrbCache.topics);
                baseTopics = baseTopics(~strcmp(baseTopics, 'message_version'));
                listStr = strjoin(sort(baseTopics), ',');
                return;
            end
            msgDir = fullfile(api.PX4Root, 'msg');
            if ~exist(msgDir, 'dir')
                error('[px4API:Error] Could not find the mandatory PX4 message root folder directory at: %s', msgDir);
            end

            files = dir(fullfile(msgDir, '**', '*.msg'));
            baseTopics = {};
            for i = 1:length(files)
                [~, camelName, ~] = fileparts(files(i).name);
                topicName = px4io.px4API.camelCaseToSnakeCase(camelName);
                if strcmp(topicName, 'message_version')
                    continue; 
                end
                baseTopics{end+1} = topicName; 
            end

            baseTopics = unique(baseTopics(~cellfun(@isempty, baseTopics)));
            listStr = strjoin(baseTopics, ',');
        end
    end
end
