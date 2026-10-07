classdef deployment
    % DEPLOYMENT Build-time services for the generated PX4 model adapter.
    %
    % configureModel is an optional, settings-only convenience method.
    % The post-code-generation callback uses the other methods to validate
    % the compiled interface, emit C/C++ code, and stage files for PX4.
    % None of these methods compiles PX4 or starts a vehicle.
    methods (Static)
        function configureModel(model)
            % CONFIGUREMODEL Apply the supported code-generation settings.
            % model is a Simulink model name. It is loaded if necessary.
            % Solver algorithm and sample time are preserved. Referenced models
            % receive the startup memory settings; other settings apply to the
            % top model. The caller chooses when to save the affected models.
            % This method does not build, export, or install callbacks.
            load_system(model);
            set_param(model, 'TargetLang', 'C', 'CodeInterfacePackaging', 'Nonreusable function', ...
                'SolverType', 'Fixed-step', 'EnableMultiTasking', 'off');
            models = find_mdlrefs(model, 'KeepModelsLoaded', true);
            for i = 1:numel(models)
                set_param(models{i}, 'ZeroInternalMemoryAtStartup', 'on', 'ZeroExternalMemoryAtStartup', 'on');
            end
        end

        function writeWrapper(model, outputDir)
            % WRITEWRAPPER Emit simulink_model_wrapper.h in outputDir.
            % Read the completed build's codeInfo.mat to obtain actual symbols
            % and the resolved base period, including FixedStep = auto.
            % Validation reports unsupported models without changing settings.
            buildDir = RTW.getBuildDir(model);
            data = load(fullfile(buildDir.BuildDirectory, 'codeInfo.mat'), 'codeInfo');
            code = data.codeInfo;
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
            w = sprintf('// Generated from codeInfo.mat; regenerate through px4io.\n#pragma once\n#include <stdint.h>\nextern "C" {\n');
            for i = 1:numel(functions)
                prototype = functions(i).Prototype;
                if ~isempty(prototype.Arguments) || ~isempty(prototype.Return)
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
end
