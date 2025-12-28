#include "common.h"

std::string str_tolower(std::string s)
{
    std::transform(s.begin(), s.end(), s.begin(),
        // static_cast<int(*)(int)>(std::tolower)         // wrong
        // [](int c){ return std::tolower(c); }           // wrong
        // [](char c){ return std::tolower(c); }          // wrong
        [](unsigned char c) { return std::tolower(c); } // correct
    );
    return s;
}

std::vector<std::string> get_file_list(const std::string& path)
{
    std::vector<std::string> m_file_list;
    if (!path.empty())
    {
        fs::path apk_path(path);
        fs::directory_iterator end;

        for (fs::directory_iterator i(apk_path); i != end; ++i)
        {
            const fs::path cp = (*i);
            m_file_list.push_back(cp.string());
        }
    }
    return m_file_list;
}

std::vector<std::string> GetFileList(const std::string& sInputhPath, const std::string& skipPath, std::vector<std::string> valid_extensions)
{
    int ret = 0;
    int deviceId = 0;		// def camera device id
    // parse path
    vector<string> filelist;

    fs::path pin(sInputhPath);
    if (fs::exists(pin))
    {
        filelist = get_file_list(pin.string());
        std::erase_if(filelist, [skipPath, valid_extensions](const std::string& filename)
            {
                bool bDeleteFromList = false;
                // delete all non image files
                // std::vector<std::string> valid_extensions = { "tif", "png", "jpg"};
                std::string file_extension = str_tolower(fs::path(filename).extension().string());

                // Entferne den Punkt am Anfang der Extension, falls vorhanden
                if (!file_extension.empty() && file_extension[0] == '.')
                    file_extension = file_extension.substr(1);

                // Prüfe, ob die Extension gültig ist
                bool is_valid_image = std::any_of(valid_extensions.begin(), valid_extensions.end(),
                    [&](const std::string& ext) {
                        return file_extension.find(ext) != std::string::npos;
                    });

                if (is_valid_image)
                {
                    // image file
                    // but exclude result files
                    if (filename.find(skipPath) != std::string::npos)
                        bDeleteFromList = true;
                }
                else
                    bDeleteFromList = true;
                return bDeleteFromList;
            });

    }
    return filelist;
}