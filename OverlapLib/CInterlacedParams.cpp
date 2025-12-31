#include "CInterlacedParams.h"

#include <fstream>
#include <sstream>
#include <iostream>
#include <unordered_map>
#include <algorithm>

// Minimal JSON helper to avoid new dependencies. Supports flat objects with numeric values.
static std::string trim(const std::string& s)
{
    auto b = s.find_first_not_of(" \t\n\r");
    if (b == std::string::npos) return "";
    auto e = s.find_last_not_of(" \t\n\r");
    return s.substr(b, e - b + 1);
}

bool CInterlacedParams::SaveToJson(const std::string& path) const
{
    std::ofstream ofs(path);
    if (!ofs)
        return false;

    ofs << "{\n";
    ofs << "  \"angle\": " << angle << ",\n";
    ofs << "  \"warp\": " << warp << ",\n";
    ofs << "  \"cuton_16\": " << cuton_16 << ",\n";
    ofs << "  \"max_expected_grayvalue\": " << max_expected_grayvalue << ",\n";
    ofs << "  \"max_mean_grayvalue16\": " << max_mean_grayvalue16 << ",\n";
    ofs << "  \"split_blobs_binary_threshold_8\": " << split_blobs_binary_threshold_8 << ",\n";
    ofs << "  \"count_lines_binary_threshold_8\": " << count_lines_binary_threshold_8 << ",\n";
    ofs << "  \"max_iter_dilate\": " << max_iter_dilate << ",\n";
    ofs << "  \"expand_bounding_box_by_num_px\": " << expand_bounding_box_by_num_px << ",\n";
    ofs << "  \"max_erode_iter\": " << max_erode_iter << ",\n";
    ofs << "  \"min_num_lines\": " << min_num_lines << ",\n";
    ofs << "  \"num_blobs_expected\": " << num_blobs_expected << ",\n";
    ofs << "  \"min_blob_x\": " << min_blob_x << ",\n";
    ofs << "  \"min_blob_y\": " << min_blob_y << ",\n";
	ofs << "  \"fResizeFactor\": " << fResizeFactor << ",\n";
    ofs << "  \"debug_output\": " << debug_output << "\n";
    ofs << "}\n";
    return true;
}

CInterlacedParams CInterlacedParams::LoadFromJson(const std::string& path)
{
    CInterlacedParams p;
    std::ifstream ifs(path);
    if (!ifs)
        return p; // return defaults

    std::string line;
    while (std::getline(ifs, line))
    {
        line = trim(line);
        if (line.empty()) continue;
        if (line.front() == '{' || line.front() == '}') continue;
        // expect "key": value[,]
        auto colon = line.find(':');
        if (colon == std::string::npos) continue;
        std::string key = trim(line.substr(0, colon));
        // remove quotes
        if (!key.empty() && key.front() == '"') key.erase(0,1);
        if (!key.empty() && key.back() == '"') key.pop_back();
        std::string valstr = trim(line.substr(colon+1));
        // remove trailing comma
        if (!valstr.empty() && valstr.back() == ',') valstr.pop_back();

        try {
            if (key == "angle") p.angle = std::stod(valstr);
            else if (key == "warp") p.warp = std::stod(valstr);
            else if (key == "cuton_16") p.cuton_16 = std::stoi(valstr);
            else if (key == "max_expected_grayvalue") p.max_expected_grayvalue = std::stoi(valstr);
            else if (key == "max_mean_grayvalue16") p.max_mean_grayvalue16 = std::stoi(valstr);
            else if (key == "split_blobs_binary_threshold_8") p.split_blobs_binary_threshold_8 = std::stoi(valstr);
            else if (key == "count_lines_binary_threshold_8") p.count_lines_binary_threshold_8 = std::stoi(valstr);
            else if (key == "max_iter_dilate") p.max_iter_dilate = std::stoi(valstr);
            else if (key == "expand_bounding_box_by_num_px") p.expand_bounding_box_by_num_px = std::stoi(valstr);
            else if (key == "max_erode_iter") p.max_erode_iter = std::stoi(valstr);
            else if (key == "min_num_lines") p.min_num_lines = std::stoi(valstr);
            else if (key == "num_blobs_expected") p.num_blobs_expected = std::stoi(valstr);
            else if (key == "min_blob_x") p.min_blob_x = std::stoi(valstr);
            else if (key == "min_blob_y") p.min_blob_y = std::stoi(valstr);
			else if (key == "fResizeFactor") p.fResizeFactor = std::stod(valstr);
            else if (key == "debug_output") p.debug_output = std::stoi(valstr);
        } catch (...) {
            // ignore parse errors, keep default
        }
    }

    return p;
}
