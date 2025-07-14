# public_graph

#### Description
An example C++ library from the Robotics, Autonomy and Planning Lab (RAP-Lab). 

#### Software Architecture
This repository consists of the following folders.
- include/ all header files.
- source/ all source files corresponding to the headers.
- test/ all test and demo files that verify and showcase the usage of the code in include/ and source/.

#### Installation and Usage


To set up and use this repository, follow these steps:

1. Clone this repository:
    ```
    git clone <repository-url>
    ```

2. Navigate to the `cpp/` directory:
    ```bash
    cd cpp/
    ```

3. Create a `build/` directory:
    ```bash
    mkdir build/
    ```

4. Create an environment variable file `.env` in the root folder with the following format:
    ```bash
    export MAPFILE="/absolute/or/relative/path/to/cbs-mapf/data/arena/arena.map"
    export SCENFILE="/absolute/or/relative/path/to/cbs-mapf/data/arena/arena.map.scen"
    ```

5. Navigate to the `build/` directory:
    ```bash
    cd build/
    ```

6. Source the `.env` file:
    ```bash
    source /absolute/or/relative/path/to/cbs-mapf/.env
    ```

7. Run CMake to configure the project:
    ```bash
    cmake ..
    ```

8. Build the project:
    ```bash
    make
    ```

9. Execute the binary with the desired input arguments:
    ```bash
    ./<binary-name> <input-args>
    ```
    
#### Acknowledgement
Credits to Zhongqiang Richard Ren and Robotics, Autonomy and Planning Lab (RAP-Lab)