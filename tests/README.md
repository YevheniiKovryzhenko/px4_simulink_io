# Metadata compatibility check

This folder contains regression checks for the metadata API used during
deployment. Keep it when upgrading MATLAB or changing wrapper generation: it
checks the two issues found during the R2026b migration, where descriptor objects
became invalid after their owner was released and empty argument collections
needed conversion before validation. It also starts from a default Simulink
model to check that `configureModel` supplies the ERT settings and can be
applied twice without changing the period or duplicating integration entries.
The configuration test verifies that empty include-directory settings stay empty
and that a user-supplied include directory with spaces remains unchanged.
A separate check rejects missing/empty message roots without changing the working
`PX4Root` value.

## Run it

With MATLAB, Simulink, and Embedded Coder available, run from the toolbox root:

```matlab
results = runtests('tests/testDeploymentMetadata.m');
assertSuccess(results);
```

Two tests should pass. A failure means root validation failed, include-directory
settings were modified, the wrapper could not be generated in the
post-code-generation callback, its metadata could not be read afterward, or its
entry points or 5,000 microsecond period were incorrect. MATLAB's failure report
identifies the failed check. Missing products or licenses can also prevent the
test from running.

The test creates a small model and generates C code in a temporary folder, then
closes the model and removes the temporary files. It does not deploy to PX4,
compile firmware, or run a simulation. Its small model deliberately isolates
metadata compatibility from message definitions and library masks.

## Full deployment check

Use `px4TestBasic` when validating the complete toolbox. Generate code with its
normal `px4io.px4API.runPostCodeGen(buildInfo)` callback and check that export
finishes successfully. This exercises the library masks, parameter bindings
(including intentionally missing parameters), uORB glue, and PX4 file export.
It replaces the configured PX4 module's generated-code directory.

The isolated test complements that workflow; it does not establish firmware
compilation or runtime behavior.
