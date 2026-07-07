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
        PX4Root = fullfile('~', 'PX4', 'v1.17.0-mod')
        PX4ModuleName = 'simulink_io'
        AllowedExtensions = {'.c', '.cpp', '.h'}        
        ShowDebug = false
    end

    properties (Access = private)
        PackageRoot = ''
        ResolvedExternalDir = ''
        LocalGeneratedDir = ''
        EnumsDir = '';
        StubHeaderName = 'px4_simulink_api.h'
        StubSrcName = 'px4_simulink_api.c'
        GlueName = 'px4_simulink_glue.cpp'
        OrbCache = struct()
        OrbCacheLoaded = false
        OrbCacheFile = 'orb_id_cache.json'
    end
    
    methods
        function obj = px4API()
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
            if obj.OrbCacheLoaded && isfield(obj.OrbCache, 'topics') && ~isempty(fieldnames(obj.OrbCache.topics))
                topicsList = fieldnames(obj.OrbCache.topics);
                for idx = 1:length(topicsList)
                    tName = topicsList{idx};
                    topicData = obj.OrbCache.topics.(tName);
                    if isfield(topicData, 'fields') && ~isempty(topicData.fields)
                        busObj = obj.createBusFromFieldData(topicData.fields);
                        if ~isempty(busObj), assignin('base', [tName, '_s'], busObj); end
                    end
                end
            end
        end
        
        function ensureParamBinding(obj, prototypeStr, implStr)
            hdrPath = fullfile(obj.LocalGeneratedDir, obj.StubHeaderName);
            srcPath = fullfile(obj.LocalGeneratedDir, obj.StubSrcName);
            if ~exist(hdrPath, 'file') || ~exist(srcPath, 'file'), obj.generateAllBussesAndHeaders(); end
            
            contentH = fileread(hdrPath);
            if ~contains(contentH, prototypeStr)
                safePrototype = sprintf('#ifdef __cplusplus\nextern "C" {\n#endif\n%s\n#ifdef __cplusplus\n}\n#endif', prototypeStr);
                fid = fopen(hdrPath, 'a'); fprintf(fid, '\n%s\n', safePrototype); fclose(fid);
            end

            contentS = fileread(srcPath);
            if ~contains(contentS, implStr)
                fid = fopen(srcPath, 'a'); fprintf(fid, '\n%s\n', implStr); fclose(fid);
            end
        end

        function ensureUorbBinding(obj, baseTopic, selectedTopic)
            % Dynamically injects uORB topic includes, structs, prototypes, and stubs.
            % Uses bulletproof string replacement to prevent duplication and structural corruption.
            
            msgDir = fullfile(obj.PX4Root, 'msg');
            msgFiles = dir(fullfile(msgDir, '**', '*.msg'));
            msgFilePath = '';
            camelName = '';
            
            % 1. Find the .msg file for the base topic
            for i = 1:length(msgFiles)
                [~, cName, ~] = fileparts(msgFiles(i).name);
                if strcmp(obj.camelCaseToSnakeCase(cName), baseTopic)
                    msgFilePath = fullfile(msgFiles(i).folder, msgFiles(i).name);
                    camelName = cName;
                    break;
                end
            end
            
            if isempty(msgFilePath)
                warning('[px4API] Could not find .msg file for %s', baseTopic);
                return;
            end

            % 2. Generate local struct string and dependencies
            [localStructStr, ~, deps, ~, ~] = obj.generateBusFromMsg(camelName, baseTopic, msgFilePath);
            
            % Recursively gather dependency structs (BFS)
            allStructs = {baseTopic, localStructStr, deps};
            queue = deps;
            
            while ~isempty(queue)
                dep = queue{1};
                queue(1) = [];
                if ~any(strcmp(allStructs(:,1), dep))
                    depFilePath = '';
                    depCamel = '';
                    for i = 1:length(msgFiles)
                        [~, cName, ~] = fileparts(msgFiles(i).name);
                        if strcmp(obj.camelCaseToSnakeCase(cName), dep)
                            depFilePath = fullfile(msgFiles(i).folder, msgFiles(i).name);
                            depCamel = cName;
                            break;
                        end
                    end
                    if ~isempty(depFilePath)
                        [depStr, ~, depDeps, ~, ~] = obj.generateBusFromMsg(depCamel, dep, depFilePath);
                        allStructs = [allStructs; {dep, depStr, depDeps}];
                        queue = [queue, depDeps];
                    end
                end
            end
            
            % Topological sort
            orderedStructs = obj.topologicalSortStructs(allStructs);
            fullLocalStructStr = '';
            for i = 1:size(orderedStructs, 1)
                fullLocalStructStr = sprintf('%s%s\n', fullLocalStructStr, orderedStructs{i, 2});
            end
            
            % 3. Build prototype and impl strings
            prototypeStr = sprintf('%s_s read_%s(void);\nvoid write_%s(%s_s in);\n%s_s init_%s(bool initialize_to_nan);', ...
                baseTopic, selectedTopic, selectedTopic, baseTopic, baseTopic, selectedTopic);
                
            implStr = sprintf('%s_s read_%s(void) { %s_s empty = {0}; return empty; }\n', baseTopic, selectedTopic, baseTopic);
            implStr = sprintf('%svoid write_%s(%s_s in) { (void)in; }\n', implStr, selectedTopic, baseTopic);
            implStr = sprintf('%s%s_s init_%s(bool initialize_to_nan) { %s_s empty = {0}; return empty; }', implStr, baseTopic, selectedTopic, baseTopic);

            % 4. Inject into header and source files using BULLETPROOF string replacement
            hdrPath = fullfile(obj.LocalGeneratedDir, obj.StubHeaderName);
            srcPath = fullfile(obj.LocalGeneratedDir, obj.StubSrcName);
            if ~exist(hdrPath, 'file') || ~exist(srcPath, 'file'), obj.generateAllBussesAndHeaders(); end
            
            % --- HEADER INJECTION ---
            contentH = fileread(hdrPath);
            
            % A. PX4 Includes
            if ~contains(contentH, sprintf('<uORB/topics/%s.h>', baseTopic))
                px4IncludeStr = sprintf('// --- DYNAMIC PX4 INCLUDES ---\n#include <uORB/topics/%s.h>\ntypedef struct %s_s %s_s;', baseTopic, baseTopic, baseTopic);
                contentH = strrep(contentH, '// --- DYNAMIC PX4 INCLUDES ---', px4IncludeStr);
            end
            
            % B. Local Structs
            if ~contains(contentH, sprintf('struct %s_s {', baseTopic))
                localStructBlock = sprintf('// --- DYNAMIC LOCAL STRUCTS ---\n%s', fullLocalStructStr);
                contentH = strrep(contentH, '// --- DYNAMIC LOCAL STRUCTS ---', localStructBlock);
            end
            
            % C. Prototypes
            if ~contains(contentH, sprintf('read_%s(void);', selectedTopic))
                protoBlock = sprintf('// --- DYNAMIC PROTOTYPES ---\n%s', prototypeStr);
                contentH = strrep(contentH, '// --- DYNAMIC PROTOTYPES ---', protoBlock);
            end
            
            % Write header back
            fid = fopen(hdrPath, 'w'); fprintf(fid, '%s', contentH); fclose(fid);
            
            % --- SOURCE INJECTION ---
            contentS = fileread(srcPath);
            if ~contains(contentS, sprintf('read_%s(void)', selectedTopic))
                fid = fopen(srcPath, 'a'); 
                fprintf(fid, '\n%s\n', implStr); 
                fclose(fid);
            end
        end

        function [needed, reason] = needsGeneration(obj)
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
                obj.clearFolderContents(obj.LocalGeneratedDir); obj.clearFolderContents(obj.EnumsDir);
                return;
            end

            msgDir = fullfile(obj.PX4Root, 'msg');
            if ~exist(msgDir, 'dir'), reason = 'PX4 msg directory is missing'; return; end

            msgFiles = dir(fullfile(msgDir, '**', '*.msg'));
            if isempty(msgFiles), needed = false; reason = 'no PX4 .msg files were found'; return; end
            
            msgFiles(contains({msgFiles.folder}, [filesep 'px4_msgs_old'])) = [];
            newestMsg = max([msgFiles(:).datenum]);

            hdrTime = dir(hdrPath).datenum; srcTime = dir(srcPath).datenum; glueTime = dir(checkPath).datenum;

            generatorFiles = {fullfile(obj.PackageRoot, 'px4API.m'), fullfile(obj.PackageRoot, 'uORB_read.m'), ...
                              fullfile(obj.PackageRoot, 'uORB_write.m'), fullfile(obj.PackageRoot, 'uORB_msg.m')};
            genTimes = [];
            for i = 1:numel(generatorFiles)
                if exist(generatorFiles{i}, 'file') == 2, genTimes(end+1) = dir(generatorFiles{i}).datenum; end
            end
            newestGenTime = max([genTimes, -inf]);

            if max([hdrTime, srcTime, glueTime]) >= max(newestMsg, newestGenTime)
                needed = false; reason = 'generated files are newer';
            else
                reason = 'dependencies are newer';
                obj.clearFolderContents(obj.LocalGeneratedDir); obj.clearFolderContents(obj.EnumsDir);
            end
        end

        function clearFolderContents(~, folderPath)
            if exist(folderPath, 'dir') == 7
                items = dir(folderPath);
                for i = 1:length(items)
                    if strcmp(items(i).name, '.') || strcmp(items(i).name, '..'), continue; end
                    fullItemPath = fullfile(items(i).folder, items(i).name);
                    if items(i).isdir, rmdir(fullItemPath, 's'); else delete(fullItemPath); end
                end
            end
        end

        function fieldMetadata = getFieldMetadataFromCache(obj, topicName)
            fieldMetadata = table();
            if isfield(obj.OrbCache, 'topics') && isfield(obj.OrbCache.topics, topicName)
                topicEntry = obj.OrbCache.topics.(topicName);
                if isfield(topicEntry, 'fields')
                    fieldsArray = topicEntry.fields;
                    if iscell(fieldsArray), fieldsArray = fieldsArray{1}; end
                    if isstruct(fieldsArray) && ~isempty(fieldsArray)
                        fieldMetadata = table({fieldsArray(:).name}', {fieldsArray(:).type}', ...
                            [fieldsArray(:).arraySize]', 'VariableNames', {'fieldName', 'fieldType', 'arraySize'});
                    end
                end
            end
        end

        function variants = getTopicVariants(obj, topicName)
            % Return the set of actual uORB topic IDs for a message base or variant.
            variants = {topicName}; % Default fallback is a cell array
            
            if ~isfield(obj.OrbCache, 'topics')
                return;
            end

            if isfield(obj.OrbCache.topics, topicName)
                entry = obj.OrbCache.topics.(topicName);
                if isfield(entry, 'variants') && ~isempty(entry.variants)
                    variants = entry.variants;
                end
            else
                % If the input is itself a variant name, return all variants of its base message.
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
            
            if ischar(variants) || isstring(variants)
                variants = {variants};
            elseif iscell(variants)
                variants = variants(~cellfun(@isempty, variants));
                if isempty(variants)
                    variants = {topicName};
                end
            end
        end

        function baseTopic = getBaseTopicForVariant(obj, topicName)
            baseTopic = topicName;
            if isfield(obj.OrbCache, 'topics')
                if isfield(obj.OrbCache.topics, topicName), return; end
                for key = fieldnames(obj.OrbCache.topics)'
                    entry = obj.OrbCache.topics.(key{1});
                    if isfield(entry, 'variants') && any(strcmp(entry.variants, topicName))
                        baseTopic = key{1}; return;
                    end
                end
            end
        end

        function saveOrbCache(obj)
            if ~exist(obj.LocalGeneratedDir, 'dir'), mkdir(obj.LocalGeneratedDir); end
            cachePath = fullfile(obj.LocalGeneratedDir, obj.OrbCacheFile);
            if ~isfield(obj.OrbCache, 'timestamp'), obj.OrbCache.timestamp = datetime('now'); end
            if ~isfield(obj.OrbCache, 'topics'), obj.OrbCache.topics = struct(); end
            fid = fopen(cachePath, 'w');
            if fid ~= -1, fprintf(fid, '%s', jsonencode(obj.OrbCache, 'PrettyPrint', true)); fclose(fid); end
        end

        function loadOrbCacheFromJson(obj)
            cachePath = fullfile(obj.LocalGeneratedDir, obj.OrbCacheFile);
            if ~exist(cachePath, 'file'), return; end
            obj.OrbCache = jsondecode(fileread(cachePath));
            if ~isfield(obj.OrbCache, 'topics'), obj.OrbCache.topics = struct(); end
            obj.OrbCacheLoaded = true;
        end

        function generateAllBussesAndHeaders(obj)
            % Scans all PX4 messages and generates a complete, static API header and source file.
            % This runs ONCE. Masks will only morph the UI, never touch the files.
            
            msgDir = fullfile(obj.PX4Root, 'msg');
            if ~exist(msgDir, 'dir'), error('[px4API:Error] Missing msg dir'); end

            rawMsgFiles = dir(fullfile(msgDir, '**', '*.msg'));
            rawMsgFiles(contains({rawMsgFiles.folder}, [filesep 'px4_msgs_old'])) = []; 

            msgFiles = []; processedTopics = {};
            for idx = 1:length(rawMsgFiles)
                [~, camelName, ~] = fileparts(rawMsgFiles(idx).name);
                topicName = obj.camelCaseToSnakeCase(camelName);
                if ~any(strcmp(processedTopics, topicName))
                    processedTopics{end+1} = topicName; 
                    msgFiles = [msgFiles; rawMsgFiles(idx)];
                end
            end

            if ~isfield(obj.OrbCache, 'topics'), obj.OrbCache.topics = struct(); end
            allStructs = {}; busAssignments = {};

            % 1. Parse all messages and build dependency graph
            for i = 1:length(msgFiles)
                msgFilePath = fullfile(msgFiles(i).folder, msgFiles(i).name);
                [~, camelName, ~] = fileparts(msgFiles(i).name);
                topicName = obj.camelCaseToSnakeCase(camelName);  
                try
                    [structString, busObj, deps, fieldMeta, topicVariants] = obj.generateBusFromMsg(camelName, topicName, msgFilePath);
                    if ~isempty(structString)
                        allStructs{end+1, 1} = topicName; 
                        allStructs{end, 2} = structString; 
                        allStructs{end, 3} = deps;
                        if ~isempty(busObj)
                            busAssignments{end+1, 1} = topicName; 
                            busAssignments{end, 2} = busObj; 
                        end
                        
                        if isempty(topicVariants), topicVariants = {topicName}; end
                        topicEntry = struct('variants', topicVariants);
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
                catch ME
                    if obj.ShowDebug, fprintf('! Skipping %s: %s\n', topicName, ME.message); end
                end
            end

            % Topological sort for local structs to prevent incomplete type errors
            if ~isempty(allStructs), allStructs = obj.topologicalSortStructs(allStructs); end

            % --- BUILD HEADER ---
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
            for i = 1:length(msgFiles)
                [~, camelName, ~] = fileparts(msgFiles(i).name);
                topicName = obj.camelCaseToSnakeCase(camelName);
                h = sprintf('%s%s_s read_%s(void);\n', h, topicName, topicName);
                h = sprintf('%svoid write_%s(%s_s in);\n', h, topicName, topicName);
                h = sprintf('%s%s_s init_%s(bool initialize_to_nan);\n', h, topicName, topicName);
            end
            h = sprintf('%suint64_t read_px4_system_time(void);\n\n', h);
            h = sprintf('%s#ifdef __cplusplus\n}\n#endif\n\n#endif // PX4_SIMULINK_API_H\n', h);

            % --- BUILD SOURCE (Desktop stubs only) ---
            % Note: NO #if guards here. Simulink's parser gets confused by them.
            % This file is excluded from PX4 builds anyway via copyGeneratedCodeFiles.
            s = sprintf('#include "%s"\n#include <math.h>\n\n', obj.StubHeaderName);
            s = sprintf('%suint64_t read_px4_system_time(void) { return 0; }\n\n', s);
            
            for i = 1:length(msgFiles)
                [~, camelName, ~] = fileparts(msgFiles(i).name);
                topicName = obj.camelCaseToSnakeCase(camelName);
                s = sprintf('%s%s_s read_%s(void) { %s_s empty = {0}; return empty; }\n', s, topicName, topicName, topicName);
                s = sprintf('%svoid write_%s(%s_s in) { (void)in; }\n', s, topicName, topicName);
                s = sprintf('%s%s_s init_%s(bool initialize_to_nan) { %s_s empty = {0}; return empty; }\n\n', s, topicName, topicName, topicName);
            end

            % Write files to disk
            if ~exist(obj.LocalGeneratedDir, 'dir'), mkdir(obj.LocalGeneratedDir); end
            fid = fopen(fullfile(obj.LocalGeneratedDir, obj.StubHeaderName), 'w'); fprintf(fid, '%s', h); fclose(fid);
            fid = fopen(fullfile(obj.LocalGeneratedDir, obj.StubSrcName), 'w'); fprintf(fid, '%s', s); fclose(fid);
            
            % Assign buses to workspace
            for i = 1:size(busAssignments, 1)
                assignin('base', [busAssignments{i, 1}, '_s'], busAssignments{i, 2});
            end
            
            obj.saveOrbCache();
            if obj.ShowDebug, fprintf('✓ Full static API generated.\n'); end
        end

        function typeSize = getPx4FieldTypeSize(~, px4Type)
            switch lower(px4Type)
                case {'uint64','int64','float64'}, typeSize = 8;
                case {'uint32','int32','float32'}, typeSize = 4;
                case {'uint16','int16'}, typeSize = 2;
                case {'uint8','int8','bool','char'}, typeSize = 1;
                otherwise, typeSize = 0;
            end
        end

        function [structStr, busObj, dependencies, fieldMetadata, topicVariants] = generateBusFromMsg(obj, camelName, topicName, msgFilePath)
            if nargin == 2, topicName = obj.camelCaseToSnakeCase(camelName); msgFilePath = ''; end
            structStr = ''; busObj = []; dependencies = {}; fieldMetadata = table(); topicVariants = {};
            if nargin < 4 || isempty(msgFilePath)
                msgFilePath = fullfile(obj.PX4Root, 'msg', [camelName, '.msg']);
                if exist(msgFilePath, 'file') ~= 2
                    fallbackFiles = dir(fullfile(obj.PX4Root, 'msg', '**', '*.msg'));
                    for fallbackIdx = 1:length(fallbackFiles)
                        [~, fallbackCamelName, ~] = fileparts(fallbackFiles(fallbackIdx).name);
                        if strcmp(fallbackCamelName, camelName)
                            msgFilePath = fullfile(fallbackFiles(fallbackIdx).folder, fallbackFiles(fallbackIdx).name); break;
                        end
                    end
                end
            end
            fid = fopen(msgFilePath, 'r');
            if fid == -1, error('[px4API:Error] Could not open message file: %s', msgFilePath); end
            lines = textscan(fid, '%s', 'Delimiter', '\n'); fclose(fid); lines = lines{1};

            structBody = sprintf('struct %s_s {\n', topicName);
            parsedFields = {}; constantsList = {}; fieldNames = {}; fieldTypes = {}; arraySizes = [];

            for i = 1:length(lines)
                line = strtrim(lines{i});
                if startsWith(line, '#')
                    stripped = regexprep(line, '^#\s*', ''); toks = strsplit(strtrim(stripped));
                    if ~isempty(toks) && strcmpi(toks{1}, 'TOPICS')
                        topicVariants = [topicVariants, toks(2:end)];
                    end
                    continue;
                end
                commentIdx = strfind(line, '#');
                if ~isempty(commentIdx), line = strtrim(line(1:commentIdx(1)-1)); end
                if contains(line, '=')
                    tokens = strsplit(line, '=');
                    if length(tokens) >= 2
                        typeAndName = strsplit(strtrim(tokens{1}));
                        if length(typeAndName) >= 2
                            constVal = strtrim(tokens{2}); if endsWith(constVal, ';'), constVal = constVal(1:end-1); end
                            constantsList{end+1, 1} = typeAndName{2}; constantsList{end, 2} = constVal;
                        end
                    end
                    continue; 
                end
                tokens = strsplit(line);
                if length(tokens) < 2, continue; end
                px4Type = tokens{1}; varName = tokens{2};
                if endsWith(varName, ';'), varName = varName(1:end-1); end
                arraySize = 1;
                arrayMatch = regexp([px4Type, ' ', varName], '\[(\d+)\]', 'tokens');
                if ~isempty(arrayMatch), arraySize = str2double(arrayMatch{1}{1}); end
                px4Type = regexprep(px4Type, '\[\d+\]', ''); varName = regexprep(varName, '\[\d+\]', '');
                if startsWith(varName, 'sl_padding'), continue; end
                [cType, slType, isDependency, depName] = obj.px4TypeToCTypesWithDeps(px4Type);
                if isDependency
                    depNameSnake = obj.camelCaseToSnakeCase(depName);
                    if ~any(strcmp(dependencies, depNameSnake)), dependencies{end+1} = depNameSnake; end
                end
                parsedFields{end+1} = struct('px4Type', px4Type, 'varName', varName, 'arraySize', arraySize, ...
                    'cType', cType, 'slType', slType, 'isDependency', isDependency, 'depName', depName);
            end

            if ~isempty(parsedFields)
                fieldSizes = cellfun(@(f) obj.getPx4FieldTypeSize(f.px4Type), parsedFields);
                [~, order] = sort(fieldSizes, 'descend'); parsedFields = parsedFields(order);
                parsedFieldsStruct = [parsedFields{:}];
                fieldNames = {parsedFieldsStruct.varName}; fieldTypes = {parsedFieldsStruct.px4Type}; arraySizes = [parsedFieldsStruct.arraySize];
            end

            for i = 1:numel(parsedFields)
                f = parsedFields{i};
                if f.arraySize > 1, structBody = sprintf('%s    %s %s[%d];\n', structBody, f.cType, f.varName, f.arraySize);
                else, structBody = sprintf('%s    %s %s;\n', structBody, f.cType, f.varName); end
            end

            if ~isempty(topicVariants), topicVariants = unique(topicVariants, 'stable'); end
            if ~isempty(constantsList)
                enumFileName = fullfile(obj.PackageRoot, '+enums', [topicName '.m']);
                efid = fopen(enumFileName, 'w');
                if efid ~= -1
                    fprintf(efid, 'classdef %s < Simulink.IntEnumType\n    enumeration\n', topicName);
                    for cIdx = 1:size(constantsList, 1), fprintf(efid, '        %s(%s)\n', constantsList{cIdx, 1}, constantsList{cIdx, 2}); end
                    fprintf(efid, '    end\nend\n'); fclose(efid); clear(topicName);
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
            fprintf('\n--- [px4API] Starting Automated Clean & Export ---\n');
            if isempty(obj.AllowedExtensions), return; end
            if ~exist(obj.PX4Root, 'dir'), error('[px4API:Error] Missing PX4 Root'); end
            if exist(obj.ResolvedExternalDir, 'dir'), rmdir(obj.ResolvedExternalDir, 's'); end
            mkdir(obj.ResolvedExternalDir);
            if ~exist(obj.LocalGeneratedDir, 'dir'), obj.generateAllBussesAndHeaders(); end

            modelName = buildInfo.getBuildName;
            obj.generateModelWrapper(modelName, obj.LocalGeneratedDir);
            obj.generateOmnipotentCppGlue(obj.LocalGeneratedDir, buildInfo);
            
            count = obj.copyGeneratedCodeFiles(obj.LocalGeneratedDir, obj.ResolvedExternalDir, false);
            buildDirInfo = RTW.getBuildDir(modelName);
            count = count + obj.copyGeneratedCodeFiles(buildDirInfo.BuildDirectory, obj.ResolvedExternalDir, false);
            
            startDirValue = buildInfo.Settings.LocalAnchorDir;
            for i = 1:length(buildInfo.ModelRefs)
                resolvedSubModelPath = strrep(buildInfo.ModelRefs(i).Path, '$(START_DIR)', startDirValue);
                count = count + obj.copyGeneratedCodeFiles(resolvedSubModelPath, obj.ResolvedExternalDir, false);
            end
            sharedUtilsDir = fullfile(startDirValue, buildDirInfo.SharedUtilsTgtDir);
            if exist(sharedUtilsDir, 'dir')
                count = count + obj.copyGeneratedCodeFiles(sharedUtilsDir, obj.ResolvedExternalDir, false);
            end
            fprintf('Successfully moved %d generated files over to PX4 code tree.\n', count);
        end

        function count = copyGeneratedCodeFiles(obj, sourceDir, destDir, recursive)
            if nargin < 4, recursive = false; end
            if ~exist(sourceDir, 'dir'), count = 0; return; end
            if ~exist(destDir, 'dir'), mkdir(destDir); end
            count = 0; entries = dir(sourceDir);
            extAllowed = cellfun(@lower, obj.AllowedExtensions, 'UniformOutput', false);
            for i = 1:length(entries)
                if entries(i).name(1) == '.', continue; end
                sourcePath = fullfile(entries(i).folder, entries(i).name);
                destPath = fullfile(destDir, entries(i).name);
                if entries(i).isdir
                    if recursive, count = count + obj.copyGeneratedCodeFiles(sourcePath, destPath, true); end
                else
                    [~, ~, ext] = fileparts(entries(i).name);
                    if strcmp(entries(i).name, obj.StubSrcName), continue; end % Skip desktop stubs
                    if ismember(lower(ext), extAllowed), copyfile(sourcePath, destPath, 'f'); count = count + 1; end
                end
            end
        end

        function generateOmnipotentCppGlue(obj, outputDir, buildInfo)
            if nargin < 2 || isempty(outputDir), outputDir = obj.ResolvedExternalDir; end
            buildDirs = buildInfo.getBuildDirList; buildName = buildInfo.getBuildName;
            testCppPath = fullfile(buildDirs{1}, sprintf('%s.c', buildName));
            
            readTopics = {}; writeTopics = {}; hasSystemTime = false;
            if isfile(testCppPath)
                testContent = fileread(testCppPath);
                readMatches = regexp(testContent, 'read_(?!px4_param_)(?!param_)(\w+)\s*\(', 'tokens');
                writeMatches = regexp(testContent, 'write_(?!px4_param_)(?!param_)(\w+)\s*\(', 'tokens');
                if ~isempty(readMatches), readTopics = unique([readMatches{:}]); end
                if ~isempty(writeMatches), writeTopics = unique([writeMatches{:}]); end
                readTopics = readTopics(~strcmp(readTopics, 'px4_system_time'));
                writeTopics = writeTopics(~strcmp(writeTopics, 'px4_system_time'));
                if contains(testContent, 'read_px4_system_time'), hasSystemTime = true; end
            else
                orbTopics = fieldnames(obj.OrbCache.topics)';
                readTopics = orbTopics; writeTopics = orbTopics; hasSystemTime = true;
            end

            hdrContent = ''; paramHdrPath = fullfile(obj.LocalGeneratedDir, obj.StubHeaderName);
            if isfile(paramHdrPath), hdrContent = fileread(paramHdrPath); end
            readParamMatches = regexp(hdrContent, '(float|int32_t)\s+read_param_(\w+)\s*\(\s*void\s*\)', 'tokens');
            writeParamMatches = regexp(hdrContent, 'void\s+write_param_(\w+)\s*\(\s*(float|int32_t)\s+value\s*\)', 'tokens');
            
            paramNames = {}; paramTypes = {};
            for i = 1:length(readParamMatches)
                cType = readParamMatches{i}{1}; name = readParamMatches{i}{2};
                if ~any(strcmp(paramNames, name)), paramNames{end+1} = name; paramTypes{end+1} = cType; end
            end
            for i = 1:length(writeParamMatches)
                name = writeParamMatches{i}{1}; cType = writeParamMatches{i}{2};
                if ~any(strcmp(paramNames, name)), paramNames{end+1} = name; paramTypes{end+1} = cType; end
            end

            cppStr = sprintf('// Auto-generated selective strongly-typed return-by-value uORB routing layer\n');
            cppStr = sprintf('%s#include <px4_platform_common/defines.h>\n#include <px4_platform_common/log.h>\n', cppStr);
            cppStr = sprintf('%s#include <uORB/uORB.h>\n#include <uORB/Publication.hpp>\n#include <uORB/Subscription.hpp>\n', cppStr);
            cppStr = sprintf('%s#include <string.h>\n#include <parameters/param.h>\n#include <uORB/topics/parameter_update.h>\n\n', cppStr);
            cppStr = sprintf('%s#include "%s"\n\n', cppStr, obj.StubHeaderName);
            if hasSystemTime, cppStr = sprintf('%s#include <drivers/drv_hrt.h>\n', cppStr); end

            allTopics = unique([readTopics, writeTopics]);
            for i = 1:length(allTopics)
                cppStr = sprintf('%s#include <uORB/topics/%s.h>\n', cppStr, obj.getBaseTopicForVariant(allTopics{i}));
            end

            cppStr = sprintf('%s\nclass SimulinkGlue {\npublic:\n\tSimulinkGlue() {}\n\t~SimulinkGlue() {}\n\n', cppStr);
            for i = 1:length(writeTopics)
                t = writeTopics{i}; b = obj.getBaseTopicForVariant(t);
                cppStr = sprintf('%s\tuORB::Publication<%s_s> _%s_pub{ORB_ID(%s)};\n', cppStr, b, t, t);
            end
            for i = 1:length(readTopics)
                t = readTopics{i}; b = obj.getBaseTopicForVariant(t);
                cppStr = sprintf('%s\tuORB::Subscription _%s_sub{ORB_ID(%s)};\n', cppStr, t, t);
            end
            cppStr = sprintf('%s};\n\nstatic SimulinkGlue g_glue_instance;\nstatic bool g_initialized = false;\n\n', cppStr);
            cppStr = sprintf('%sextern "C" {\n\n', cppStr);

            if hasSystemTime
                cppStr = sprintf('%suint64_t read_px4_system_time(void) {\n\treturn hrt_absolute_time();\n}\n\n', cppStr);
            end

            for i = 1:length(readTopics)
                t = readTopics{i}; b = obj.getBaseTopicForVariant(t);
                cppStr = sprintf('%s%s_s read_%s(void) {\n\tstatic %s_s local_buffer{};\n', cppStr, b, t, b);
                cppStr = sprintf('%s\tif (g_initialized) g_glue_instance._%s_sub.copy(&local_buffer);\n', cppStr, t);
                cppStr = sprintf('%s\treturn local_buffer;\n}\n\n', cppStr);
            end
            for i = 1:length(allTopics)
                t = allTopics{i}; b = obj.getBaseTopicForVariant(t);
                if ~strcmp(t, b) && ~any(strcmp(readTopics, b))
                    cppStr = sprintf('%s%s_s read_%s(void) { return read_%s(); }\n\n', cppStr, b, b, t);
                end
            end

            for i = 1:length(writeTopics)
                t = writeTopics{i}; b = obj.getBaseTopicForVariant(t);
                cppStr = sprintf('%svoid write_%s(%s_s in) {\n\tif (g_initialized) g_glue_instance._%s_pub.publish(in);\n}\n\n', cppStr, t, b, t);
            end
            for i = 1:length(allTopics)
                t = allTopics{i}; b = obj.getBaseTopicForVariant(t);
                if ~strcmp(t, b) && ~any(strcmp(writeTopics, b))
                    cppStr = sprintf('%svoid write_%s(%s_s in) { write_%s(in); }\n\n', cppStr, b, b, t);
                end
            end

            for i = 1:length(allTopics)
                b = allTopics{i}; variants = obj.getTopicVariants(b);
                for v = 1:length(variants)
                    vName = variants{v};
                    cppStr = sprintf('%s%s_s init_%s(bool initialize_to_nan) {\n\tstruct %s_s msg;\n\tmemset(&msg, 0, sizeof(msg));\n', cppStr, b, vName, b);
                    cppStr = sprintf('%s\tif (initialize_to_nan) {\n', cppStr);
                    fieldMeta = obj.getFieldMetadataFromCache(b);
                    if ~isempty(fieldMeta) && height(fieldMeta) > 0
                        for fIdx = 1:height(fieldMeta)
                            if strcmp(fieldMeta.fieldType{fIdx}, 'float32') || strcmp(fieldMeta.fieldType{fIdx}, 'float64')
                                fName = fieldMeta.fieldName{fIdx}; aSize = fieldMeta.arraySize(fIdx);
                                if aSize > 1
                                    for idx = 0:(aSize-1), cppStr = sprintf('%s\t\tmsg.%s[%d] = NAN;\n', cppStr, fName, idx); end
                                else
                                    cppStr = sprintf('%s\t\tmsg.%s = NAN;\n', cppStr, fName);
                                end
                            end
                        end
                    end
                    cppStr = sprintf('%s\t}\n\treturn msg;\n}\n\n', cppStr);
                end
                if ~any(strcmp(variants, b))
                    cppStr = sprintf('%s%s_s init_%s(bool initialize_to_nan) { return init_%s(initialize_to_nan); }\n\n', cppStr, b, b, variants{1});
                end
            end

            if ~isempty(paramNames)
                cppStr = sprintf('%s// --- Zero-Overhead Parameter Bridge ---\n', cppStr);
                cppStr = sprintf('%stypedef struct {\n', cppStr);
                for i = 1:length(paramNames), cppStr = sprintf('%s    %s %s;\n', cppStr, paramTypes{i}, paramNames{i}); end
                cppStr = sprintf('%s} simulink_params_t;\n\n', cppStr);
                cppStr = sprintf('%sstatic simulink_params_t g_simulink_params = {};\n', cppStr);
                cppStr = sprintf('%sstatic param_t g_param_handles[%d] = {};\n', cppStr, length(paramNames));
                cppStr = sprintf('%sstatic uORB::Subscription g_param_update_sub{ORB_ID(parameter_update)};\n\n', cppStr);
                
                cppStr = sprintf('%sstatic void init_simulink_params() {\n', cppStr);
                for i = 1:length(paramNames)
                    cppStr = sprintf('%s    g_param_handles[%d] = param_find("%s");\n', cppStr, i-1, upper(paramNames{i}));
                end
                cppStr = sprintf('%s}\n\n', cppStr);
                
                cppStr = sprintf('%sstatic void update_simulink_params_impl() {\n', cppStr);
                cppStr = sprintf('%s    if (g_param_update_sub.updated()) {\n', cppStr);
                cppStr = sprintf('%s        parameter_update_s update;\n        g_param_update_sub.copy(&update);\n', cppStr);
                for i = 1:length(paramNames)
                    cppStr = sprintf('%s        if (g_param_handles[%d] != PARAM_INVALID) param_get(g_param_handles[%d], &g_simulink_params.%s);\n', cppStr, i-1, i-1, paramNames{i});
                end
                cppStr = sprintf('%s    }\n}\n\n', cppStr);
                
                for i = 1:length(paramNames)
                    name = paramNames{i}; cType = paramTypes{i};
                    cppStr = sprintf('%s%s read_param_%s(void) { return g_simulink_params.%s; }\n\n', cppStr, cType, name, name);
                    cppStr = sprintf('%svoid write_param_%s(%s value) {\n', cppStr, name, cType);
                    if strcmp(cType, 'float')
                        cppStr = sprintf('%s    if (!PX4_ISFINITE(g_simulink_params.%s) || fabsf(g_simulink_params.%s - value) > 1e-6f) {\n', cppStr, name, name);
                    else
                        cppStr = sprintf('%s    if (g_simulink_params.%s != value) {\n', cppStr, name);
                    end
                    cppStr = sprintf('%s        g_simulink_params.%s = value;\n', cppStr, name);
                    cppStr = sprintf('%s        if (g_param_handles[%d] != PARAM_INVALID) param_set(g_param_handles[%d], &value);\n', cppStr, i-1, i-1);
                    cppStr = sprintf('%s    }\n}\n\n', cppStr);
                end
            end

            cppStr = sprintf('%svoid update_simulink_params(void) {\n', cppStr);
            if ~isempty(paramNames), cppStr = sprintf('%s    update_simulink_params_impl();\n', cppStr); end
            cppStr = sprintf('%s}\n\n', cppStr);

            cppStr = sprintf('%svoid init_px4_simulink_io(void) {\n\tg_initialized = true;\n', cppStr);
            if ~isempty(paramNames)
                cppStr = sprintf('%s\tinit_simulink_params();\n\tupdate_simulink_params_impl();\n', cppStr);
            end
            cppStr = sprintf('%s}\n\n}\n', cppStr);

            if ~exist(outputDir, 'dir'), mkdir(outputDir); end
            fid = fopen(fullfile(outputDir, obj.GlueName), 'w');
            if fid == -1, error('[px4API:Error] Could not write glue file'); end
            fprintf(fid, '%s', cppStr); fclose(fid);
        end
    end

    methods (Static)
        function resolvedPath = resolveAbsolutePath(pathStr)
            resolvedPath = pathStr;
            if startsWith(resolvedPath, '~/') || strcmp(resolvedPath, '~')
                resolvedPath = fullfile(getenv('HOME'), resolvedPath(2:end));
            elseif startsWith(resolvedPath, './') || startsWith(resolvedPath, '../')
                resolvedPath = char(java.io.File(resolvedPath).getCanonicalPath());
            end
        end

        function snakeName = camelCaseToSnakeCase(camelName)
            snakeName = lower(regexprep(camelName, '([a-z0-9])([A-Z])', '$1_$2'));
        end

        function variants = extractMsgTopicVariants(msgFilePath, defaultTopicName)
            variants = {defaultTopicName};
            if ~exist(msgFilePath, 'file'), return; end
            fid = fopen(msgFilePath, 'r');
            if fid == -1, return; end
            lines = textscan(fid, '%s', 'Delimiter', '\n'); fclose(fid); lines = lines{1};
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
            if isempty(variants), variants = {defaultTopicName}; end
        end

        function [cType, slType, isDependency, depName] = px4TypeToCTypesWithDeps(px4Type)
            isDependency = false; depName = '';
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
                    isDependency = true; depName = px4Type;
                    structNameSnake = px4io.px4API.camelCaseToSnakeCase(px4Type);
                    slType = [structNameSnake, '_s']; cType = ['struct ', structNameSnake, '_s'];
            end
        end

        function orderedStructs = topologicalSortStructs(allStructs)
            if isempty(allStructs), orderedStructs = {}; return; end
            numStructs = size(allStructs, 1);
            struct_map = containers.Map();
            for i = 1:numStructs, struct_map(allStructs{i, 1}) = i; end
            visited = false(numStructs, 1); orderedStructs = {};
            for i = 1:numStructs
                if ~visited(i)
                    [orderedStructs, visited] = px4io.px4API.dfs_visit(i, allStructs, struct_map, visited, orderedStructs);
                end
            end
        end

        function [orderedStructs, visited] = dfs_visit(idx, allStructs, struct_map, visited, orderedStructs)
            if visited(idx), return; end
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
            api = px4io.px4API();
            files = dir(fullfile(api.PX4Root, 'msg', '**', '*.msg'));
            topics = {};
            for i = 1:length(files)
                [~, camelName, ~] = fileparts(files(i).name);
                topicName = px4io.px4API.camelCaseToSnakeCase(camelName);
                if strcmp(topicName, 'message_version'), continue; end
                variants = px4io.px4API.extractMsgTopicVariants(fullfile(files(i).folder, files(i).name), topicName);
                topics = [topics, variants];
            end
            listStr = strjoin(unique(topics(~cellfun(@isempty, topics))), ',');
        end

        function busObj = createBusFromFieldData(fieldsData)
            if isempty(fieldsData), busObj = []; return; end
            elements = [];
            for fIdx = 1:length(fieldsData)
                if iscell(fieldsData), f = fieldsData{fIdx}; else f = fieldsData(fIdx); end
                if isfield(f, 'varName'), fName = f.varName; fType = f.slType;
                else
                    fName = f.name;
                    [~, fType, ~, ~] = px4io.px4API.px4TypeToCTypesWithDeps(f.type);
                end
                elem = Simulink.BusElement;
                elem.Name = fName; elem.DataType = fType; elem.Dimensions = f.arraySize;
                elem.Complexity = 'real'; elem.SampleTime = -1; elem.DimensionsMode = 'Fixed';
                elements = [elements; elem];
            end
            busObj = Simulink.Bus;
            busObj.Elements = elements; busObj.DataScope = 'Imported'; busObj.HeaderFile = 'px4_simulink_api.h';
        end

        function runPostCodeGen(buildInfo)
            apiInstance = px4io.px4API();
            apiInstance.exportGeneratedCode(buildInfo);
        end

        function uorb_topic_callback(callbackContext)
            blockHandle = callbackContext.BlockHandle;
            choices = strsplit(px4io.px4API.getTopicDropdownString(), ',');
            set_param(blockHandle, 'TypeOptions_uorb_topic', choices);
        end

        function generateModelWrapper(modelName, outputDir)
            if isempty(modelName), error('modelName empty'); end
            isLoaded = bdIsLoaded(modelName); if ~isLoaded, load_system(modelName); end
            hasInputs = ~isempty(find_system(modelName, 'SearchDepth', 1, 'BlockType', 'Inport'));
            hasOutputs = ~isempty(find_system(modelName, 'SearchDepth', 1, 'BlockType', 'Outport'));
            if ~isLoaded, close_system(modelName, 0); end

            w = sprintf('// Auto-generated model-agnostic wrapper\n#pragma once\n\n');
            w = sprintf('%s#ifdef __cplusplus\nextern "C" {\n#endif\n', w);
            w = sprintf('%svoid init_px4_simulink_io(void);\nvoid update_simulink_params(void);\n', w);
            w = sprintf('%svoid %s_initialize(void);\nvoid %s_step(void);\n', w, modelName, modelName);
            w = sprintf('%s#ifdef __cplusplus\n}\n#endif\n\n#include "%s.h"\n\n', w, modelName);
            w = sprintf('%snamespace SimulinkWrapper {\nclass SimulinkModel {\npublic:\n', w);
            w = sprintf('%s    void initialize() { init_px4_simulink_io(); %s_initialize(); }\n', w, modelName);
            w = sprintf('%s    void step() { %s_step(); }\n', w, modelName);
            if hasInputs, w = sprintf('%s    ExtU_%s_T& getExternalInputs() { return %s_U; }\n', w, modelName, modelName);
            else, w = sprintf('%s    void* getExternalInputs() { return nullptr; }\n', w); end
            if hasOutputs, w = sprintf('%s    const ExtY_%s_T& getExternalOutputs() { return %s_Y; }\n', w, modelName, modelName);
            else, w = sprintf('%s    void* getExternalOutputs() { return nullptr; }\n', w); end
            w = sprintf('%s};\n}  // namespace SimulinkWrapper\n', w);

            if ~exist(outputDir, 'dir'), mkdir(outputDir); end
            fid = fopen(fullfile(outputDir, 'simulink_model_wrapper.h'), 'w');
            fprintf(fid, '%s', w); fclose(fid);
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
                topicName = px4io.px4API.camelCaseToSnakeCase(camelName);

                % Skip internal metadata framework tags
                if strcmp(topicName, 'message_version'), continue; end

                % Add only the base topic name, not variants
                baseTopics{end+1} = topicName; %#ok<AGROW>
            end

            % Deduplicate across subfolders and sort alphabetically
            baseTopics = unique(baseTopics(~cellfun(@isempty, baseTopics)));
            listStr = strjoin(baseTopics, ',');
        end
    end
end