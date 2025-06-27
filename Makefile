# Variables for build directory and build type
build_dir="build"  # Directory where build files will be generated
flag="Debug"       # Default build type

# Default target that depends on the 'build' target
all: build

# Targets 'fast' and 'dev' both depend on the 'all' target
fast dev: all

# Override the 'flag' variable for the 'dev' and 'fast' targets
dev: flag = Debug   # Development build uses Debug mode
fast: flag = Release # Fast build uses Release mode

# Declare 'gen', 'build', and 'clean' as phony targets (not associated with files)
.PHONY: gen build clean

# Target to generate build files using CMake
gen:
	@mkdir -p ${build_dir}  # Create the build directory if it doesn't exist
	@echo "cmake -B${build_dir} -H. -DCMAKE_BUILD_TYPE=${flag} -DCMAKE_EXPORT_COMPILE_COMMANDS=1"
	@eval "cmake -B${build_dir} -H. -DCMAKE_BUILD_TYPE=${flag} -DCMAKE_EXPORT_COMPILE_COMMANDS=1"

# Target to build the project
build: gen
	@echo "cd ${build_dir} && make -j"  # Navigate to the build directory and build using Make
	@eval "cd ${build_dir} && make -j"

# Target to clean the build directory
clean:
	@echo "cd ${build_dir} && make clean"  # Navigate to the build directory and clean it
	@eval "cd ${build_dir} && make clean"
