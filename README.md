# PBCount

Research source code release for pseudo boolean counter for our papers: 
  
*Engineering an Exact Pseudo-Boolean Model Counter* (AAAI2024)
  
*Towards Projected and Incremental Pseudo-Boolean Model Counting* (AAAI2025)

## Dependencies

cudd 3.0.0 (```https://github.com/ivmai/cudd/releases/tag/cudd-3.0.0```)  
cxxopts 2.1.2 (```https://github.com/jarro2783/cxxopts/blob/v2.1.2/include/cxxopts.hpp```)  
boost 1.82 (```https://www.boost.org/```)

## Build

Make sure cmake and boost library is installed on the system.

Please build with gcc 10 and above for linux and clang for macOS.

Pleas download `lib.tar` from `https://github.com/vardigroup/ADDMC/blob/master/lib.tar` and place it in the main codebase directory.

Alternatively you can create the `lib.tar` in the main directory of codebase. `lib.tar` should contain `cudd-3.0.0` and `cxxopts.hpp` and extracts to the `lib` folder as follows:
```
<main codebase dir>
    - lib
        - cudd-3.0.0 (extract the cudd files here, in the same directory structure as the repo)
        - cxxopts.hpp
```

`lib.tar` is automatically extracted using cmake, which is invoked by `./COMPILE.sh`.

```
Build `pbcount` with `./COMPILE.sh`
```

If you have the library files in other directories, please make changes to the `CMakeLists.txt` file using `LINK_DIRECTORIES` to specify library location and `INCLUDE_DIRECTORIES` to specify include directories for builds. 

## Usage

Example pb formula file (`.opb` files) can be found in the `examples` folder.

Call pbcount on the examples as follow

```
./pbcount --wf 1 --cf examples/example1.opb
./pbcount --wf 1 --cf examples/example2.opb
./pbcount --wf 2 --cf examples/example3.opb
```

For more detailed usage and paramters, see `./pbcount -h`