# PX4 Simulink I/O toolbox

This toolbox connects generated Simulink C code to PX4 uORB messages, parameters,
and the PX4 clock. It generates the interface code and exports a model into
`src/modules/simulink_io/generated_code` for compilation with PX4 firmware.

The handwritten `simulink_io` task schedules the generated adapter and reports
performance counters. Model-specific names, sample periods, message routing,
parameter bindings, and lifecycle calls belong to the generator. Flight behavior
such as arming, offboard requests, takeoff, and control remains in the model.

## Components and responsibilities

| Component | Responsibility |
| --- | --- |
| `px4_lib.slx` | Blocks for reading, writing, and initializing messages, accessing parameters, and reading system time |
| `+px4io/px4API.m` | Message discovery, buses, enums, static API, parameter bindings, hardware glue, and export |
| `+px4io/deployment.m` | Optional model settings helper, compiled interface validation, adapter generation, and export utilities |
| `+px4io/generated_code` | Local generated API, desktop stubs, message cache, and parameter manifest |
| `px4/src/modules/simulink_io` | Matching handwritten PX4 runner, build configuration, and parameter definitions |
| PX4 `src/modules/simulink_io/generated_code` | Exported model sources and generated hardware adapter/glue |

Generated files are build products. Make persistent interface changes in the
toolbox and algorithm changes in the model, then regenerate. Use the matching
runner and generator versions together.

## Requirements and initial setup

- MATLAB, Simulink, and Embedded Coder for C code generation.
- A compatible PX4 source tree containing the messages the model uses.
- A configured PX4 build toolchain for the selected SITL or hardware target.

Add this repository to the MATLAB path, open `px4_lib.slx`, and use its blocks in
your model. The supplied `Test` and `px4Test*` models provide examples.

Set `PX4Root` and `PX4ModuleName` in `+px4io/px4API.m` for your PX4 checkout and
module directory. Defaults point to `~/PX4/v1.17.0-mod` and `simulink_io`.
Install the matching module template in that PX4 checkout and enable it in the
selected board configuration. Export requires the module directory to exist.

The block callbacks initialize the API through `px4io.px4API.getInstance()`.
API initialization checks generated artifacts against message and generator
modification times, generates missing or outdated artifacts, and restores bus
objects in the MATLAB base workspace. The singleton reuses those objects during
model editing.

For explicit API initialization, use:

```matlab
api = px4io.px4API.getInstance();
```

## Configure and save a model once

Configure these settings manually in Simulink, or use the optional helper:

```matlab
px4io.deployment.configureModel('Test');
```

| Setting | Required value |
| --- | --- |
| Target language (`TargetLang`) | C |
| Code interface (`CodeInterfacePackaging`) | Nonreusable function |
| Solver type (`SolverType`) | Fixed-step |
| Multitasking (`EnableMultiTasking`) | Off |
| Internal startup memory initialization (`ZeroInternalMemoryAtStartup`) | On |
| External startup memory initialization (`ZeroExternalMemoryAtStartup`) | On |

The helper changes only model configuration. It does not generate code, export
files, compile PX4, or save the model. It preserves the selected solver algorithm
and base sample time. It enables the two startup initialization settings on
referenced models as well; other helper settings apply to the top model.

Save the affected models in Simulink to retain these settings. Once saved, normal
model edits and PX4 firmware updates do not require running the helper again.
Use it again only for a new model or to restore settings that have changed.
Manual configuration is equally supported.

Configure the model's **Post-code generation command** (`PostCodeGenCommand`) as:

```matlab
px4io.px4API.runPostCodeGen(buildInfo);
```

The settings helper does not install this callback or configure the model's
custom-code paths. Keep the library/example model's custom-code integration and
the callback configured, and save them with the model.

## Everyday workflow: Generate Code

1. Edit the model normally.
2. Press **Generate Code** in Simulink.
3. Let the configured post-code-generation callback export the model.
4. Build the selected PX4 firmware target and run SITL or deploy the firmware.

There is no separate configuration or adapter-generation command to run on each
iteration. `slbuild('Test')` is a command-line alternative to the Simulink build
workflow; it is not an additional step after pressing Generate Code. The model's
build settings determine whether Simulink generates code only or also invokes
its own toolchain.

The callback automatically:

1. Collects current parameter bindings from the root and referenced models.
2. Reads `codeInfo.mat` to validate the compiled interface and generate the
   adapter's entry points and resolved sample period.
3. Scans model, referenced-model, and shared-utility source files for uORB reads,
   writes, message initializers, and system-time calls.
4. Generates the hardware glue required by those calls.
5. Collects model sources and headers, excludes desktop stubs and example main
   programs, and checks for missing files or conflicting filenames.
6. Replaces the deployed generated-code directory only after preparing a complete
   staging directory. A failed replacement attempts to restore the old directory.

Export does not compile PX4, start a simulator, flash hardware, arm a vehicle, or
change runtime PX4 parameters. Build and deployment errors must be resolved before
using the resulting firmware. If export fails, the previous deployed code remains
in place; a successful firmware build alone does not prove the new model exported.

## What to do after changes

| Change | Required action |
| --- | --- |
| Model logic, blocks, topics, or parameter bindings | Generate Code, then rebuild PX4 |
| Model sample time | Generate Code, then rebuild PX4; no manual C++ period edit |
| New model or changed code-generation settings | Configure manually or use the helper, save, then Generate Code |
| PX4 implementation changes with unchanged message interfaces | Rebuild PX4; no need to reapply model settings |
| PX4 `.msg` definitions or selected PX4 source tree | Refresh the message API and buses **before** generating model code, then regenerate and rebuild |
| Toolbox generator or runner interface | Keep the toolbox and runner matched, regenerate model code, then rebuild PX4 |

A model must be compiled against the message layouts for its target PX4 version.
After changing `.msg` definitions in an existing MATLAB session, refresh the API:

```matlab
px4io.px4API.forceGenerateAll();
```

Then update the model and use Generate Code. If the C Caller block retains an old
function definition, refresh its custom-code interface or reopen the model before
rebuilding. Switching PX4 checkouts may require adapting blocks when messages or
fields were renamed or removed.

This refresh is separate from model configuration. The singleton does not
continuously watch PX4 files, and the post-code-generation callback cannot
retroactively recompile a model's bus layouts. Message-interface preparation must
happen before model code generation; model-specific adapter and glue generation
happen automatically afterward.

## Supported generated interface and timing

The runner accepts fixed-step, single-tasking, nonreusable C code with one
initialization function and one combined periodic `void step(void)` function.
A termination function is optional. Separate output/update entry points, argument-
based entry points, multiple independently scheduled step functions, and nonzero
base-period offsets are not supported by this adapter.

The generator obtains entry-point names and timing from `codeInfo.mat`. It uses
the resolved period even when the model's `FixedStep` setting is `auto`. Periods
must be positive whole microseconds representable by a 32-bit unsigned integer.
Slower rates may be scheduled within the generated combined step function.

Both startup memory initialization options are required so module stop/start
resets model state rather than relying on memory being zeroed only when PX4 starts.
The export validator reports unsupported settings without modifying them.

## uORB messages and desktop simulation

The static C API contains base messages and their declared topic variants. Bus
objects and enums are generated from PX4 `.msg` definitions. Keeping the complete
API available lets C Caller blocks resolve functions during model editing.
Hardware glue is generated for the functions used by the current build, including
messages used only through an initializer.

- **Read:** returns the latest received message. Before the first publication,
  fields are zero. Subsequent reads retain the last message if there is no update.
  Check timestamps in the model when validity or freshness matters.
- **Write:** publishes a message to the selected topic. Populate timestamps and
  other required fields in the model.
- **Initialize:** zeroes the message and, when requested, sets its floating-point
  fields to NaN. Select the message or topic variant in the block.
- **System time:** returns the PX4 high-resolution clock in microseconds.

Topic variants use the base message's structure type. Select an explicit variant
to address a specific topic. The base-name compatibility path maps to the first
declared topic when the base name has no matching ORB_ID. A topic variant is not
a dynamically selected uORB multi-instance index.

Read buffers reset when the module initializes; subscriptions are released on
normal termination. Desktop readers return zero messages, writers do nothing,
and the desktop clock returns zero. Desktop initializers follow the same zero/NaN
policy as hardware. These stubs support the Simulink interface but do not emulate
PX4 sensor data, timing, vehicle dynamics, or flight behavior.

## Parameters

Parameter blocks identify PX4 parameters by name and type. The generator collects
bindings into a manifest and rebuilds that manifest during export, including
referenced models. The bridge resolves parameter handles at initialization,
caches values, and refreshes them on `parameter_update` notifications. Reads use
the cache; writes use the PX4 parameter API and update the cache on success.

Parameter names must exist in the selected firmware with compatible types.
Bindings do not create parameter definitions automatically. Maintain definitions
through the project's parameter-generation tools or module YAML files and rebuild
PX4 when definitions change. Missing or mismatched parameters produce diagnostics;
float reads return NaN and integer reads return zero when unavailable. Model logic
must decide how to handle unavailable values.

## PX4 runner and firmware build

```sh
make px4_sitl_simulink
```

CMake monitors the generated `.c` and `.cpp` file set so a renamed or replaced model
updates the compiled sources. The exported desktop stub `px4_simulink_api.c` and
generated example main programs are excluded from the PX4 build.

At the PX4 shell:

```sh
simulink_io start
simulink_io status
simulink_io stop
```

The task initializes the adapter, runs it at its generated period, and terminates
it on normal stop. `status` reports the model, configured period, execution-time,
interval, and overrun counters. An overrun resets the scheduling baseline without
replaying missed steps. The task does not impose arming, offboard, takeoff, or
estimator-validity gates. The model and other PX4 modules own those decisions.

Task priority and stack size are platform settings in the handwritten runner.
Model computation must fit its execution period and available resources. SITL
lockstep timing requires simulator time to advance; a standalone lifecycle test
without a simulator does not measure flight-loop timing.

See [the module README](px4/src/modules/simulink_io/README.md) for board variants,
airframes, and the distinction between stock-controller and full-Simulink control.

## Troubleshooting

| Symptom | Check |
| --- | --- |
| Adapter lacks `period_us`, `name`, or `terminate` | Use matching toolbox and module versions and regenerate the model; ensure MATLAB is using the intended toolbox path |
| Export reports unsupported interface or restart settings | Correct and save the model settings, then Generate Code again |
| Topic or field is missing in Simulink | Refresh the API for the selected PX4 messages before code generation and refresh the C Caller interface |
| Parameter-not-found or type-mismatch warning | Check the parameter definition in the selected firmware and the block's name/type |
| Export reports a filename collision | Resolve conflicting generated/custom source names; export intentionally refuses to overwrite different files with the same destination name |
| Firmware builds but runs the previous model | Check that the post-code-generation callback completed and that `PX4Root` points to the firmware checkout being built |
| Overruns or unexpected timing | Inspect `simulink_io status`, the compiled sample period, model execution cost, and simulator clock behavior |

# Contact
If you have any questions, please feel free to contact me, Yevhenii (Jack) Kovryzhenko, at yevhenii.kovryzhenko@okstate.edu.

# Credit
This work started during my Ph.D. at [ACELAB](https://etaheri0.wixsite.com/acelabauburnuni) at Auburn University, under the supervision of Dr. Ehsan Taheri.

I am still in the process of publishing journal and conference papers that have directly used this work, so I will keep this section actively updated. Feel free to credit [me](https://scholar.google.com/citations?user=P812qiUAAAAJ&hl=en) by citing any of my relevant works.
Some of the articles that directly used this work:
* Kovryzhenko, Yevhenii, and Ehsan Taheri. 2027. “Comparison of Control Allocation Algorithms for eVTOL Aircraft: Application to Tiltwings.” Paper presented at AIAA SciTech Forum. AIAA SciTech Forum, January 11.
    ```bibtex
    @inproceedings{kovryzhenko_comparison_2027,
        address = {Orlando, FL},
        title = {Comparison of {Control} {Allocation} {Algorithms} for {eVTOL} {Aircraft}: {Application} to {Tiltwings}},
        language = {en},
        booktitle = {{AIAA} {SciTech} {Forum}},
        publisher = {American Institute of Aeronautics and Astronautics},
        author = {Kovryzhenko, Yevhenii and Taheri, Ehsan},
        month = jan,
        year = {2027},
    }
    ```
* Kovryzhenko, Yevhenii, and Ehsan Taheri. 2027. “Feedback Control for eVTOL Aircraft: Multirotor, Fixed-Wing, and Tilt-Wing Architectures.” Paper presented at AIAA SciTech Forum. AIAA SciTech Forum, January 11.
    ```bibtex
    @inproceedings{kovryzhenko_feedback_2027,
        address = {Orlando, FL},
        title = {Feedback {Control} for {eVTOL} {Aircraft}: {Multirotor}, {Fixed}-{Wing}, and {Tilt}-{Wing} {Architectures}},
        language = {en},
        booktitle = {{AIAA} {SciTech} {Forum}},
        publisher = {American Institute of Aeronautics and Astronautics},
        author = {Kovryzhenko, Yevhenii and Taheri, Ehsan},
        month = jan,
        year = {2027},
    }
    ```
* Kovryzhenko, Yevhenii, and Ehsan Taheri. 2027. “Unified Motion Planning of Quadrotors, Fixed-Wing, and Tiltwing eVTOL Aircraft with Differential Flatness.” Paper presented at AIAA SciTech Forum. AIAA SciTech Forum, January 11.
    ```bibtex
    @inproceedings{kovryzhenko_unified_2027,
        address = {Orlando, FL},
        title = {Unified {Motion} {Planning} of {Quadrotors}, {Fixed}-{Wing}, and {Tiltwing} {eVTOL} {Aircraft} with {Differential} {Flatness}},
        language = {en},
        booktitle = {{AIAA} {SciTech} {Forum}},
        publisher = {American Institute of Aeronautics and Astronautics},
        author = {Kovryzhenko, Yevhenii and Taheri, Ehsan},
        month = jan,
        year = {2027},
    }
    ```
* Kovryzhenko, Yevhenii. 2026. “Unified Guidance & Control Framework for eVTOL Aircraft.” Dissertation, Auburn University. https://etd.auburn.edu/handle/10415/10559
    ```bibtex
    @phdthesis{kovryzhenko_unified_2026,
        type = {Dissertation},
        title = {Unified {Guidance} \& {Control} {Framework} for {eVTOL} {Aircraft}},
        copyright = {EMBARGO\_GLOBAL},
        url = {https://etd.auburn.edu/handle/10415/10559},
        language = {en},
        urldate = {2026-08-05},
        school = {Auburn University},
        author = {Kovryzhenko, Yevhenii},
        month = aug,
        year = {2026}
    }
    ```