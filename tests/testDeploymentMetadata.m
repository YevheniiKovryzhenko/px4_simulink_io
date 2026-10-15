function tests = testDeploymentMetadata
% TESTDEPLOYMENTMETADATA Catch MATLAB release changes in generated metadata.
% Run from the toolbox root:
%   results = runtests('tests/testDeploymentMetadata.m');
%   assertSuccess(results);
%
% Requires MATLAB, Simulink, and Embedded Coder. Creates a temporary model
% and generates C code; it does not load px4TestBasic or export anything to
% the PX4 checkout. See tests/README.md for intent and expected results.
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
% Keep the toolbox available while the test works in a temporary folder.
% The fixture restores the original MATLAB path when the suite finishes.
root = fileparts(fileparts(mfilename('fullpath')));
testCase.applyFixture(matlab.unittest.fixtures.PathFixture(root));
end

function testInvalidPX4RootDoesNotReplaceWorkingConfiguration(testCase)
% A bad path must fail before it can invalidate caches or affect a build.
api = px4io.px4API.getInstance();
original = api.PX4Root;
folder = tempname;
mkdir(folder);
cleanupFolder = onCleanup(@() rmdir(folder, 's'));
verifyError(testCase, @() assignRoot(api, fullfile(folder, 'missing')), 'px4io:InvalidPX4Root');
verifyError(testCase, @() assignRoot(api, folder), 'px4io:InvalidPX4Root');
mkdir(fullfile(folder, 'msg'));
verifyError(testCase, @() assignRoot(api, folder), 'px4io:InvalidPX4Root');
verifyError(testCase, @() assignRoot(api, fullfile(original, 'msg')), 'px4io:InvalidPX4Root');
verifyEqual(testCase, api.PX4Root, original);
% Dot components should normalize to the same root, not a new relative path.
api.PX4Root = fullfile(original, '.');
verifyEqual(testCase, api.PX4Root, original);
previousDirectory = pwd;
cleanupDirectory = onCleanup(@() cd(previousDirectory));
cd(original);
api.PX4Root = '.';
verifyEqual(testCase, api.PX4Root, original);
end

function assignRoot(api, value)
% MATLAB anonymous functions cannot contain a property assignment.
api.PX4Root = value;
end

function testPostCodeGenReadsMetadataAndWritesFiveMillisecondWrapper(testCase)
% A simple passthrough model isolates the metadata API from PX4 messages,
% parameter bindings, and library masks. Its 5 ms period is easy to verify
% in both the Simulink metadata (seconds) and PX4 wrapper (microseconds).
% Fixtures remove the temporary build and restore the working directory;
% onCleanup closes this model without saving further changes, even on error.
testCase.applyFixture(matlab.unittest.fixtures.WorkingFolderFixture);
% A user's session may direct generated code outside the working folder.
% Isolate both build/cache paths here and restore them when the test ends.
previous = Simulink.fileGenControl('getConfig');
previousCache = previous.CacheFolder;
previousCode = previous.CodeGenFolder;
cleanupPaths = onCleanup(@() Simulink.fileGenControl('set', ...
    'CacheFolder', previousCache, 'CodeGenFolder', previousCode));
Simulink.fileGenControl('set', 'CacheFolder', fullfile(pwd, 'cache'), ...
    'CodeGenFolder', fullfile(pwd, 'build'), 'createDir', true);
model = 'px4MetadataProbe';
new_system(model);
cleanupModel = onCleanup(@() close_system(model, 0));
add_block('simulink/Sources/In1', [model '/Input']);
add_block('simulink/Sinks/Out1', [model '/Output']);
add_line(model, 'Input/1', 'Output/1');
% Start with the normal Simulink target: the helper must supply the ERT
% integration settings instead of relying on an already configured example.
set_param(model, 'Solver', 'FixedStepDiscrete', 'FixedStep', '0.005', ...
    'CustomHeaderCode', '#include <math.h>', ...
    'PostCodeGenCommand', 'disp(''custom callback preserved'');');
% Include directories belong to the user. Empty settings must stay empty;
% toolbox files are found through the API's existing MATLAB path setup.
px4io.deployment.configureModel(model);
verifyEmpty(testCase, get_param(model, 'SimUserIncludeDirs'));
verifyEmpty(testCase, get_param(model, 'CustomInclude'));
% A user-supplied include directory (including spaces) must also be untouched.
userInclude = fullfile(pwd, 'user includes');
mkdir(userInclude);
quotedUserInclude = ['"' userInclude '"'];
set_param(model, 'SimUserIncludeDirs', quotedUserInclude, 'CustomInclude', quotedUserInclude);
px4io.deployment.configureModel(model);
expectedIncludes = quotedUserInclude;
verifyEqual(testCase, get_param(model, 'SimUserIncludeDirs'), expectedIncludes);
verifyEqual(testCase, get_param(model, 'CustomInclude'), expectedIncludes);
verifyEqual(testCase, get_param(model, 'SimUserSources'), 'px4_simulink_api.c');
verifyEmpty(testCase, get_param(model, 'CustomSource'));
% Reapplying the helper must preserve timing and avoid duplicate callbacks
% or custom-code entries when the user runs setup again.
callbackBefore = get_param(model, 'PostCodeGenCommand');
includesBefore = get_param(model, 'CustomInclude');
px4io.deployment.configureModel(model);
verifyEqual(testCase, get_param(model, 'PostCodeGenCommand'), callbackBefore);
verifyEqual(testCase, get_param(model, 'CustomInclude'), includesBefore);
verifyEqual(testCase, get_param(model, 'FixedStep'), '0.005');
verifyTrue(testCase, contains(callbackBefore, 'px4io.px4API.runPostCodeGen(buildInfo)'));
verifyTrue(testCase, contains(callbackBefore, 'custom callback preserved'));
verifyTrue(testCase, contains(get_param(model, 'CustomHeaderCode'), '#include <math.h>'));
outputDir = fullfile(pwd, 'wrapper');
% Generate the wrapper during PostCodeGenCommand, where the real deployment
% reads metadata. Checking only after slbuild could miss callback timing bugs.
callback = sprintf('px4io.deployment.writeWrapper(''%s'',''%s'');', ...
    model, strrep(outputDir, '''', ''''''));
set_param(model, 'PostCodeGenCommand', callback);
save_system(model);
slbuild(model);
buildDirectory = RTW.getBuildDir(model);
code = px4io.deployment.readCodeInterface(buildDirectory.BuildDirectory);
% Regression: returned function objects become invalid if the helper releases
% their descriptor owner. Read them after returning to catch that failure.
verifyNumElements(testCase, code.OutputFunctions, 1);
verifyEqual(testCase, code.OutputFunctions.Prototype.Name, [model '_step']);
verifyEqual(testCase, code.OutputFunctions.Timing.SamplePeriod, 0.005);
wrapper = fileread(fullfile(outputDir, 'simulink_model_wrapper.h'));
% Regression: an empty descriptor argument list is still a Sequence object.
% These declarations prove the wrapper accepted valid void(void) functions.
% Also check the seconds-to-microseconds conversion used by the PX4 task.
verifyTrue(testCase, contains(wrapper, 'period_us = 5000;'));
verifyTrue(testCase, contains(wrapper, 'void px4MetadataProbe_step(void);'));
verifyTrue(testCase, contains(wrapper, 'void px4MetadataProbe_initialize(void);'));
end
