#include <iostream>
#include <apriltag.h>
#include <tag16h5.h>

int main(int argc, char* argv[]) {
    apriltag_family_t* fam = tag16h5_create();

    std::cout << "Tag Name: " << fam->name << "\n";

    tag16h5_destroy(fam);
}