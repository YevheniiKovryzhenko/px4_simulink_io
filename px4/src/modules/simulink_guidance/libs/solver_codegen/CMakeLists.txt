#project(TRAJECTORY_SOLVER)
#file(GLOB TRS_SOURCES "*.h" "*.c"
#        "CXSparse/.*h" "CXSparse/.*c"
#        "CXSparse/CXSparseSupport/.*h" "CXSparse/CXSparseSupport/.*c"
#        "CXSparse/Include/.*h" "CXSparse/Include/.*c"
#        "CXSparse/Source/.*h" "CXSparse/Source/.*c"
#        "CXSparse/SuiteSparse_config/.*h" "CXSparse/SuiteSparse_config/.*c"
#        "interface/.*h" "interface/.*c")

#target_include_directories(trajectory_solver PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")

set(HEADERS
include/coder_cs.h
include/coder_posix_time.h
include/cs.h
include/makeCXSparseMatrix.h
include/rtGetInf.h
include/rtGetNaN.h
include/rt_nonfinite.h
include/rtwtypes.h
include/solve_from_lu.h
include/solve_from_qr.h
include/solver_codegen_emxAPI.h
include/solver_codegen_emxutil.h
include/solver_codegen.h
include/solver_codegen_types.h
include/SuiteSparse_config.h
include/tmwtypes.h
include/unpackCXStruct.h)

set(SOURCES
src/coder_posix_time.c
src/cs_ci_bundle.c
src/cs_ri_bundle.c
src/makeCXSparseMatrix.c
src/rtGetInf.c
src/rtGetNaN.c
src/rt_nonfinite.c
src/solve_from_lu.c
src/solve_from_qr.c
src/solver_codegen.c
src/solver_codegen_emxAPI.c
src/solver_codegen_emxutil.c
src/unpackCXStruct.c)
#find ./ -type f -print0 | xargs -0 mv -t ./
#mv *.h include/
#mv *.c src/

px4_add_library(trajectory_solver STATIC ${HEADERS} ${SOURCES})

#target_link_libraries(trajectory_solver
#	PRIVATE
#		px4_work_queue
#		sensor_calibration
#)

#add_library(
#    trajectory_solver STATIC ${HEADERS} ${SOURCES})
target_include_directories(trajectory_solver PUBLIC include)
target_include_directories(trajectory_solver PUBLIC include)

