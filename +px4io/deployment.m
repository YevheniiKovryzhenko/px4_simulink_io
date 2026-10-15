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

classdef deployment
    % DEPLOYMENT Build-time services for the generated PX4 model adapter.
    %
    % configureModel prepares model settings and the toolbox API integration.
    % The post-code-generation callback uses the other methods to validate
    % the compiled interface, emit C/C++ code, and stage files for PX4.
    % None of these methods compiles PX4 or starts a vehicle.
    methods (Static)
        function configureModel(model)
            % CONFIGUREMODEL Prepare a model and its references for PX4 export.
            % Uses portable C99 ERT code for SITL and 32-bit flight controllers.
            % Installs the top-model export callback and adds the toolbox's
            % custom-code integration without replacing unrelated custom code.
            % Fixed-step solver choice, base period, explicit data types, and
            % explicit parameter storage classes remain model-owned. Simulink
            % selects a compatible solver when converting variable-step models.
            % Initializes toolbox buses/headers but does not build, deploy, or
            % save models. Save each changed model after reviewing settings.
            px4io.px4API.getInstance();
            load_system(model);
            model = get_param(model, 'Name');
            apiDirectory = fullfile(fileparts(mfilename('fullpath')), 'generated_code');
            models = find_mdlrefs(model, 'KeepModelsLoaded', true);
            for i = 1:numel(models)
                current = models{i};
                % Select ERT first: switching target can reset other settings.
                set_param(current, 'SystemTargetFile', 'ert.tlc');
                set_param(current, 'HardwareBoard', 'None', ...
                    'TargetLang', 'C', 'TargetLangStandard', 'C99 (ISO)', ...
                    'ProdHWDeviceType', 'ARM Compatible->ARM Cortex-M', ...
                    'ProdLongLongMode', 'on', 'ProdEqTarget', 'on', ...
                    'PortableWordSizes', 'on', ...
                    'DataTypeReplacement', 'CDataTypesFixedWidth', ...
                    'CodeReplacementLibrary', 'None');
                % PX4 owns compilation, scheduling, and the process entry point.
                set_param(current, 'SolverType', 'Fixed-step', ...
                    'EnableMultiTasking', 'off', 'GenCodeOnly', 'on', ...
                    'GenerateMakefile', 'off', 'GenerateSampleERTMain', 'off', ...
                    'MatFileLogging', 'off', 'GenerateAllocFcn', 'off', ...
                    'IncludeMdlTerminateFcn', 'on', 'SupportNonFinite', 'on', ...
                    'SupportContinuousTime', 'on', ...
                    'MATLABDynamicMemAlloc', 'off', ...
                    'ZeroInternalMemoryAtStartup', 'on', ...
                    'ZeroExternalMemoryAtStartup', 'on');
                % Storage/code-size optimizations; retain IEEE NaNs and startup
                % resets used by missing parameters, message fields, and restart.
                set_param(current, 'DefaultParameterBehavior', 'Inlined', ...
                    'BlockReduction', 'on', 'ExpressionFolding', 'on', ...
                    'OptimizeBlockIOStorage', 'on', 'BufferReuse', 'on', ...
                    'LocalBlockOutputs', 'on', 'EnableMemcpy', 'on');
                % Desktop stubs belong only to simulation. Firmware receives
                % the generated hardware bridge through the export callback.
                set_param(current, 'SimTargetLang', 'C', ...
                    'SimUseLocalCustomCode', 'on', 'SimParseCustomCode', 'on', ...
                    'RTWUseSimCustomCode', 'off', 'RTWUseLocalCustomCode', 'on');
                px4io.deployment.appendSetting(current, 'SimCustomHeaderCode', '#include "px4_simulink_api.h"');
                px4io.deployment.appendSetting(current, 'CustomHeaderCode', '#include "px4_simulink_api.h"');
                px4io.deployment.appendSetting(current, 'SimUserIncludeDirs', ['"' apiDirectory '"']);
                px4io.deployment.appendSetting(current, 'CustomInclude', ['"' apiDirectory '"']);
                px4io.deployment.appendSetting(current, 'SimUserSources', 'px4_simulink_api.c');
            end
            % Referenced models retain their own function packaging; only the
            % top model provides the no-argument entry points called by PX4.
            set_param(model, 'CodeInterfacePackaging', 'Nonreusable function', ...
                'CombineOutputUpdateFcns', 'on');
            callback = get_param(model, 'PostCodeGenCommand');
            if isempty(regexp(callback, 'px4io\.px4API\.runPostCodeGen\s*\(\s*buildInfo\s*\)', 'once'))
                px4io.deployment.appendSetting(model, 'PostCodeGenCommand', ...
                    'px4io.px4API.runPostCodeGen(buildInfo);');
            end
        end

        function writeWrapper(model, outputDir)
            % WRITEWRAPPER Emit simulink_model_wrapper.h in outputDir.
            % Read the completed build's interface metadata for actual symbols
            % and the resolved base period, including FixedStep = auto.
            % Validation reports unsupported models without changing settings.
            buildDir = RTW.getBuildDir(model);
            code = px4io.deployment.readCodeInterface(buildDir.BuildDirectory);
            if ~strcmp(get_param(model, 'TargetLang'), 'C') || ...
                    ~strcmp(get_param(model, 'CodeInterfacePackaging'), 'Nonreusable function') || ...
                    ~strcmp(get_param(model, 'SolverType'), 'Fixed-step') || ...
                    ~strcmp(get_param(model, 'EnableMultiTasking'), 'off')
                error('px4io:UnsupportedInterface', 'Run px4io.deployment.configureModel(''%s'') before rebuilding: PX4 requires fixed-step, single-tasking, nonreusable C.', model);
            end
            models = find_mdlrefs(model, 'KeepModelsLoaded', true);
            for i = 1:numel(models)
                if ~strcmp(get_param(models{i}, 'ZeroInternalMemoryAtStartup'), 'on') || ...
                        ~strcmp(get_param(models{i}, 'ZeroExternalMemoryAtStartup'), 'on')
                    error('px4io:RestartInitialization', 'Enable both startup memory initialization options on %s and rebuild (px4io.deployment.configureModel).', models{i});
                end
            end
            if numel(code.OutputFunctions) ~= 1 || numel(code.InitializeFunctions) ~= 1 || ...
                    ~isempty(code.UpdateFunctions) || numel(code.TerminateFunctions) > 1
                error('px4io:UnsupportedInterface', 'Expected one initialize function and one combined periodic step function.');
            end
            % A single PX4 task calls the combined step function at this rate.
            timing = code.OutputFunctions.Timing;
            if numel(timing) ~= 1 || ~strcmp(timing.TimingMode, 'PERIODIC') || timing.SampleOffset ~= 0
                error('px4io:UnsupportedTiming', 'Expected a periodic step with zero offset.');
            end
            % PX4 schedules in whole microseconds. Permit only rounding noise,
            % not a fractional period that would change the model's timing.
            us = timing.SamplePeriod * 1e6;
            if ~isscalar(us) || ~isfinite(us) || us < 1 || us > double(intmax('uint32')) || abs(us-round(us)) > 1e-6
                error('px4io:UnsupportedTiming', 'Base period must be a positive whole number of microseconds.');
            end
            functions = [code.InitializeFunctions(:); code.OutputFunctions(:); code.TerminateFunctions(:)];
            % Declare only validated entry points. Including native uORB headers
            % inside extern C would give their C++ print_message overloads C linkage.
            w = sprintf('// Generated from Simulink code interface metadata; regenerate through px4io.\n#pragma once\n#include <stdint.h>\nextern "C" {\n');
            for i = 1:numel(functions)
                prototype = functions(i).Prototype;
                % Arguments are a Sequence object even when empty.
                inputArgs = prototype.Arguments.toArray();
                if ~isempty(inputArgs) || ~isempty(prototype.Return)
                    error('px4io:UnsupportedInterface', 'Entry points must have void(void) signatures.');
                end
                w = sprintf('%svoid %s(void);\n', w, prototype.Name);
            end
            w = sprintf('%svoid init_px4_simulink_io(void);\nvoid update_simulink_params(void);\nvoid terminate_px4_simulink_io(void);\n}\nnamespace SimulinkWrapper {\nclass SimulinkModel {\npublic:\n', w);
            w = sprintf('%s    static constexpr uint32_t period_us = %.0f;\n', w, round(us));
            w = sprintf('%s    static constexpr const char *name() { return "%s"; }\n', w, model);
            w = sprintf('%s    void initialize() { init_px4_simulink_io(); %s(); }\n', w, code.InitializeFunctions.Prototype.Name);
            w = sprintf('%s    void step() { update_simulink_params(); %s(); }\n', w, code.OutputFunctions.Prototype.Name);
            % Termination is optional in the generated model; the I/O bridge
            % always receives its termination call.
            terminate = '';
            if ~isempty(code.TerminateFunctions)
                terminate = sprintf('%s(); ', code.TerminateFunctions.Prototype.Name);
            end
            w = sprintf('%s    void terminate() { %sterminate_px4_simulink_io(); }\n};\n}\n', w, terminate);
            if ~isfolder(outputDir)
                mkdir(outputDir);
            end
            fid = fopen(fullfile(outputDir, 'simulink_model_wrapper.h'), 'w');
            if fid < 0
                error('px4io:WriteFailed', 'Cannot write generated adapter.');
            end
            cleanup = onCleanup(@() fclose(fid));
            fprintf(fid, '%s', w);
        end

        function code = readCodeInterface(buildDirectory)
            % READCODEINTERFACE Read the public Code Descriptor API.
            if ~isfile(fullfile(buildDirectory, 'codedescriptor.dmr'))
                error('px4io:MissingCodeInterface', ...
                    'No codedescriptor.dmr in build folder %s. Regenerate model code before exporting.', buildDirectory);
            end
            descriptor = coder.getCodeDescriptor(buildDirectory);
            % Function interfaces refer into the descriptor's data model.
            % Keep its owner alive while the caller consumes those objects.
            code.Descriptor = descriptor;
            code.InitializeFunctions = descriptor.getFunctionInterfaces('Initialize');
            code.OutputFunctions = descriptor.getFunctionInterfaces('Output');
            code.UpdateFunctions = descriptor.getFunctionInterfaces('Update');
            code.TerminateFunctions = descriptor.getFunctionInterfaces('Terminate');
        end

        function files = sourceFiles(buildInfo)
            % SOURCEFILES Return existing model and dependency source paths.
            % BuildInfo supplies referenced-model and shared-utility sources.
            % Exclude desktop stubs, previously generated glue, and example
            % main programs, which must not be linked into the PX4 module.
            files = buildInfo.getFullFileList('source');
            files = unique(files(:)', 'stable');
            keep = true(size(files));
            excluded = {'px4_simulink_api.c', 'px4_simulink_glue.cpp', 'ert_main.c', 'ert_main.cpp', 'rt_main.c'};
            for i = 1:numel(files)
                [~, n, e] = fileparts(files{i});
                keep(i) = ~ismember([n e], excluded);
                if keep(i) && ~isfile(files{i})
                    error('px4io:MissingSource', 'Missing generated source: %s', files{i});
                end
            end
            files = files(keep);
            if isempty(files)
                error('px4io:MissingSource', 'No generated model sources.');
            end
        end

        function [reads, writes, inits, hasTime] = discoverCalls(files)
            % DISCOVERCALLS Collect API call suffixes from C/C++ source files.
            % Outputs are unique read/write/init names and a system-time flag.
            % Ignore comments, strings, and parameter calls. This is a lexical
            % scan, not a C parser; the caller filters names against known topics.
            reads = {};
            writes = {};
            inits = {};
            hasTime = false;
            for i = 1:numel(files)
                content = regexprep(fileread(files{i}), '/\*[\s\S]*?\*/|//[^\r\n]*|"(?:\\.|[^"\\])*"', ' ');
                calls = regexp(content, '\<(read|write|init)_(\w+)\s*\(', 'tokens');
                for j = 1:numel(calls)
                    kind = calls{j}{1};
                    topic = calls{j}{2};
                    if strcmp(topic, 'px4_system_time')
                        hasTime = true;
                    elseif startsWith(topic, 'param_') || startsWith(topic, 'px4_param_')
                        continue;
                    elseif strcmp(kind, 'read')
                        reads{end + 1} = topic;
                    elseif strcmp(kind, 'write')
                        writes{end + 1} = topic;
                    else
                        inits{end + 1} = topic;
                    end
                end
            end
            reads = unique(reads);
            writes = unique(writes);
            inits = unique(inits);
        end

        function text = initializer(base, topic, fields)
            % INITIALIZER Return a C message-initialization function as text.
            % base is the structure type, topic is the API suffix, and fields
            % is a metadata table with fieldName, fieldType, and arraySize.
            % Start with zero bytes; optionally replace direct float32/float64
            % fields (including arrays) with NaN. Shared by desktop and PX4 code.
            text = sprintf('%s_s init_%s(bool initialize_to_nan) {\n    %s_s msg;\n    memset(&msg, 0, sizeof(msg));\n    if (initialize_to_nan) {\n', base, topic, base);
            for i = 1:height(fields)
                if ismember(fields.fieldType{i}, {'float32', 'float64'})
                    if fields.arraySize(i) > 1
                        text = sprintf('%s        for (unsigned i = 0; i < %d; ++i) msg.%s[i] = NAN;\n', text, fields.arraySize(i), fields.fieldName{i});
                    else
                        text = sprintf('%s        msg.%s = NAN;\n', text, fields.fieldName{i});
                    end
                end
            end
            text = sprintf('%s    }\n    return msg;\n}\n\n', text);
        end

        function copyFile(source, dest)
            % COPYFILE Copy one source without hiding a filename collision.
            % Identical existing text is accepted. Different contents at the
            % same destination raise an error instead of silently overwriting.
            if isfile(dest)
                if ~strcmp(fileread(source), fileread(dest))
                    error('px4io:FilenameCollision', 'Conflicting exported filename: %s', dest);
                end
            else
                copyfile(source, dest);
            end
        end

        function replaceDirectory(stage, target)
            % REPLACEDIRECTORY Install a fully prepared staging directory.
            % Move the current export aside first. If installation fails,
            % attempt to restore it and propagate the error. Keep stage and
            % target on the same filesystem; no concurrency locking is provided.
            backup = [tempname(fileparts(target)) '_backup'];
            hadTarget = isfolder(target);
            if hadTarget
                movefile(target, backup);
            end
            try
                movefile(stage, target);
            catch err
                if hadTarget
                    movefile(backup, target);
                end
                rethrow(err);
            end
            if hadTarget
                rmdir(backup, 's');
            end
        end

        function cleanupDirectory(directory)
            % CLEANUPDIRECTORY Remove a temporary directory if it still exists.
            % Used by onCleanup after either successful export or an error.
            if isfolder(directory)
                rmdir(directory, 's');
            end
        end
    end

    methods (Static, Access = private)
        function appendSetting(model, parameter, entry)
            % Add toolbox integration once, retaining the user's custom code.
            existing = get_param(model, parameter);
            if ~contains(existing, entry)
                if isempty(strtrim(existing))
                    updated = entry;
                else
                    updated = sprintf('%s\n%s', existing, entry);
                end
                set_param(model, parameter, updated);
            end
        end
    end
end
