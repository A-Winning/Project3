#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>

int main() {
    std::ifstream infile("US.txt");
    std::ofstream outfile("US_trimmed.txt"); // Output file

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

        // Ensure enough fields
        if (tokens.size() >= 17 && (tokens[6] == "P" || tokens[6] == "L")) {
            try {
                float lat = std::stof(tokens[4]);
                float lon = std::stof(tokens[5]);

                if (lat > 24.5169 && lat < 49.3333 && lon < -66.96 && lon > -124.7167) {
                    outfile << tokens[1] << "\t";  // name

                    if (tokens[6] == "P") {
                        outfile << tokens[16] << "\t"; // population only for P
                    } else {
                        outfile << "-" << "\t"; // placeholder for L
                    }

                    outfile << tokens[4] << "\t"     // latitude
                            << tokens[5] << "\t"     // longitude
                            << tokens[6] << "\n";    // feature class
                }
            } catch (...) {
                continue; // skip invalid lat/lon rows
            }
        }
    }

    infile.close();
    outfile.close();

    std::cout << "Filtered Ps and Ls into US_trimmed.txt\n";
    return 0;
}
