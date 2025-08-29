#!/bin/bash

# rm -rf build bin

mkdir build

cd build

cmake -DCMAKE_BUILD_TYPE=Release ..

# cmake -DCMAKE_BUILD_TYPE=Debug ..

make -j$(nproc)

cd ..

# ./bin/cvrp instance/P-n20-k2.vrp -u 230

# ./bin/cvrp instance/B/B-n57-k9.vrp -u 1757

# ./bin/cvrp instance/XML10/XML10_1111_03.vrp -u 4800

# ./bin/cvrp instance/P/P-n19-k2.vrp -u 230

# ./bin/cvrp instance/P/P-n16-k8.vrp -u 450

# ./bin/cvrp instance/XML10_2111_01.vrp -u 4000

# ./bin/cvrp instance/XML10/XML10_1111_01.vrp -u 4950

# ./bin/cvrp instance/XML25/XML25_1111_01.vrp -u 11086

# ./bin/cvrp instance/XML25/XML25_1111_02.vrp -u 11626

# ./bin/cvrp instance/XML25/XML25_1111_03.vrp -u 11624

# ./bin/cvrp instance/XML25/XML25_1111_14.vrp -u 9803

# ./bin/cvrp instance/XML15/XML15_1111_02.vrp -u 6768

# ./bin/cvrp instance/XML15/XML15_1111_01.vrp -u 6879


# "./bin/cvrp instance/XML15/XML15_1111_03.vrp -u 6647"
# "./bin/cvrp instance/XML15/XML15_1111_04.vrp -u 7922"
# "./bin/cvrp instance/XML15/XML15_1111_05.vrp -u 7654"
# "./bin/cvrp instance/XML15/XML15_1111_06.vrp -u 5879"
# "./bin/cvrp instance/XML15/XML15_1111_07.vrp -u 6866"
# "./bin/cvrp instance/XML15/XML15_1111_08.vrp -u 7110"
# "./bin/cvrp instance/XML15/XML15_1111_09.vrp -u 6461"
# "./bin/cvrp instance/XML15/XML15_1111_10.vrp -u 6430"
# "./bin/cvrp instance/XML15/XML15_1111_11.vrp -u 6950"
# "./bin/cvrp instance/XML15/XML15_1111_12.vrp -u 6714"
# "./bin/cvrp instance/XML15/XML15_1111_13.vrp -u 8374"
# "./bin/cvrp instance/XML15/XML15_1111_14.vrp -u 5828"

# gdb --args ./bin/cvrp instance/XML15_varDemand/XML15_1121_01.vrp -u 6982

./bin/cvrp instance/XML15_varDemand/XML15_1121_01.vrp -u 6982
