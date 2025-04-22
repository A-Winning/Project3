//
// Created by alexa on 22/04/2025.
//
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>

int main() {
    std::ifstream infile("US.txt");
    std::ofstream outfile("US_trimmed.txt"); // New filtered file

    if (!infile.is_open() || !outfile.is_open()) {
        std::cerr << "Failed to open input or output file.\n";
        return 1;
    }

    std::string line;
    while (std::getline(infile, line)) {
        std::stringstream ss(line);
        std::string token;
        std::vector<std::string> tokens;

        // Split by tab
        while (std::getline(ss, token, '\t')) {
            tokens.push_back(token);
        }

        // Feature class is in column 7 (index 6)
        if (tokens.size() >= 7 && (tokens[6] == "P" || tokens[6] == "L")) {
            outfile << tokens[1] + "\t" + tokens[4] + "\t" + tokens[5] + "\t" + tokens[6] << "\n";
        }
    }

    infile.close();
    outfile.close();

    std::cout << "Filtered file written to US_trimmed.txt\n";
    return 0;
}