# PX4 Simulink Code Generator

## Hello there 👋

This MATLAB/Simulink framework automates the generation of C code and Simulink bus definitions for seamless PX4 uORB topic integration and parameter access. It bridges the gap between Simulink control algorithms and the PX4 autopilot firmware, allowing developers to deploy complex models directly into the PX4 ecosystem with minimal manual C++ coding.

### Key Features
- **Static API:** Generates a complete, static C header/source file for all uORB topics to prevent Simulink C Caller cache invalidation issues.
- **Parameters:** Extracts parameter usage directly from the Simulink model and generates a highly optimized C++ bridge using cached `param_t` handles and uORB boolean flags.
- **PX4 Native Math:** Uses `PX4_ISFINITE` and `fabsf` to satisfy strict PX4 compiler flags (`-Werror=float-equal`) without relying on the C++ `std::` namespace.
- **Automated Bus & Struct Generation:** Parses PX4 `.msg` files, resolves dependencies via topological sorting, and automatically creates matching Simulink Bus objects and C structs with correct memory alignment.

# Integration with PX4
This tool is designed to integrate directly into your custom PX4 firmware tree. It scans your local PX4 repository's `msg` directory to automatically map uORB topics and parameters to Simulink. 

This project is the next generation of now obsolete [PX4 Simulink I/O Framework](https://github.com/YevheniiKovryzhenko/PX4_SIMULINK_IO_Framework.git). You may also find [KGroundControl](https://github.com/YevheniiKovryzhenko/KGroundControl.git) useful for building out the rest of your deployment and communication pipeline.

# Installation 
The core functionality lives in the `+px4io/` folder, with the vast majority of the logic located in `+px4io/px4api.m`. To use the project in your simulink model, open the px4_lib.slx library and copy the relevant blocks from there. The px4Test* simulink models are simple usage examples.

### Prerequisites
- MATLAB / Simulink with Embedded Coder.
- A local clone of the PX4 firmware repository (the generator requires access to the `msg` folder to parse uORB topics).

### Configuration
1. Open `+px4io/px4api.m`.
2. Update the `PX4Root` property to point to your local PX4 firmware repository root (e.g., `fullfile('~', 'PX4', 'v1.17.0-mod')`).
3. Update the `PX4ModuleName` property to match your target PX4 module directory (e.g., `'simulink_io'`).

### Generating the API
Simply instantiate the class in the MATLAB Command Window. It will automatically check file timestamps and regenerate the C/C++ headers, source stubs, and Simulink buses if any PX4 `.msg` files or generator scripts have changed.
```matlab
api = px4io.px4API();
```
When you use the provided blocks from `px4_lib.slx`, no extra configurtion steps are necessary simply drag and drop those blocks into your model and everything should be automatically generated when you click on "generate code" button in simulink.

### Simulink Code Generation
The `px4API` class hooks into Simulink's post-code generation callbacks (`runPostCodeGen`). When you build your Simulink model for deployment:
1. The generator purges the target PX4 generated code directory.
2. It generates a model-agnostic C++ wrapper and the hardware-specific uORB/Param routing layer (`px4_simulink_glue.cpp`).
3. It automatically copies all necessary `.c`, `.cpp`, and `.h` files into your PX4 firmware tree (`src/modules/<your_module>/generated_code`).
4. Desktop simulation stubs (`px4_simulink_api.c`) are intentionally excluded from the hardware build to prevent multiple-definition linker errors on the PX4 target.

# Contact
If you have any questions, please feel free to contact me, Yevhenii (Jack) Kovryzhenko, at yzk0058@auburn.edu.

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