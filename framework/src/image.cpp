#include "image.h"
// Suppress warnings in third-party code.
#include <framework/disable_all_warnings.h>
DISABLE_WARNINGS_PUSH()
#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>
DISABLE_WARNINGS_POP()
#include <cassert>
#include <exception>
#include <iostream>
#include <string>
#include <glm/glm.hpp>
#include <glm/gtx/string_cast.hpp>

using namespace std;
Image::Image(const std::filesystem::path& filePath)
{
    if (!std::filesystem::exists(filePath)) {
        std::cerr << "Texture file " << filePath << " does not exists!" << std::endl;
        throw std::exception();
    }

    const auto filePathStr = filePath.string(); // Create l-value so c_str() is safe.
    int numChannels;
    stbi_uc* pixels = stbi_load(filePathStr.c_str(), &m_width, &m_height, &numChannels, STBI_rgb);

    if (numChannels < 3) {
        std::cerr << "Only textures with 3 or more color channels are supported. " << filePath << " has " << numChannels << " channels" << std::endl;
        throw std::exception();
    }
    if (!pixels) {
        std::cerr << "Failed to read texture " << filePath << " using stb_image.h" << std::endl;
        throw std::exception();
    }

    //std::cout << "Number of channels in texture: " << numChannels << std::endl;
    for (size_t i = 0; i < m_width * m_height * numChannels; i += numChannels) {
        m_pixels.emplace_back(pixels[i + 0] / 255.0f, pixels[i + 1] / 255.0f, pixels[i + 2] / 255.0f);
    }

    stbi_image_free(pixels);
}

glm::vec3 Image::getTexel(const glm::vec2& textureCoordinates) const
{
    int x = round(textureCoordinates.x * m_width);
    int y = round(textureCoordinates.y * m_height);
    x = max(0, x);
    y = max(0, y);
    x = min(x, m_width - 1);
    y = min(y, m_height - 1);
    //const glm::ivec2 pixel = glm::ivec2(textureCoordinates * glm::vec2(m_width, m_height) + 0.5f);
    //glm::vec3 result = m_pixels[pixel.y * m_width + pixel.x];
    glm::vec3 result = m_pixels[y * m_width + x];

    //cout << glm::to_string(result) << endl;
    return result;
}
