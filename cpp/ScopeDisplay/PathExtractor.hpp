#pragma once

#include "../pugixml/src/pugixml.hpp"
#include <iostream>
#include <string>

class PathExtractor
{
public:
    PathExtractor(char* aFilename)
    {
        pugi::xml_document doc;
        pugi::xml_parse_result result = doc.load_file(aFilename);
    }
};
