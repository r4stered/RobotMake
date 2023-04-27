#include <iostream>
#include <wpi/interpolating_map.h>
#include <wpi/timestamp.h>

int main(int argc, char* argv[]) {
    std::cout << "Hello Rio!\n";
    wpi::interpolating_map<double, double> map;

    std::cout << "Time since epoch: " << wpi::NowDefault() << "\n";

    map.insert(3, 0);
    map.insert(4, 10);
    std::cout << "Result: " << map[3.5] << "\n";
    return 0;
}