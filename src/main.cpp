#include <iostream>
#include <ntcore.h>

int main(int argc, char* argv[]) {
    auto myValue = nt::GetEntry(nt::GetDefaultInstance(), "MyValue");
    nt::SetEntryValue(myValue, nt::Value::MakeString("Hello World"));
    std::cout << nt::GetEntryValue(myValue).GetString() << "\n";
    return 0;
}