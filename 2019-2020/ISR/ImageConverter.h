#ifndef IMAGECONVERTER_H
#define IMAGECONVERTER_H

#include <cstdlib>
#include <string>
#include <map>

#define mwidth 100
#define maxstring 210
#define maxColorValue 255

class ImageConverter
{
private:
	char charSet[90] = {'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n', 'o', 'p',
						'q', 'r', 's', 't', 'u', 'v', 'w', 'x', 'y', 'z', 'A', 'B', 'C', 'D', 'E', 'F',
						'G', 'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P', 'Q', 'R', 'S', 'T', 'U', 'V',
						'W', 'X', 'Y', 'Z', '!', '@', '#', '$', '%', '^', '&', '*', '(', ')', '-', '_',
						'+', '=', '{', '[', '}', ']', ':', ';', '<', ',', '>', '.', '~', '`', '?', '/'};
	char intToCharMap[maxColorValue];
	int col[3][100];
	std::map<char,int> charmap;
public:
	ImageConverter();
	std::string stringBuffer[300] = {};
	int size_stringBuffer = 0;
	int imageMatrix[3][mwidth][mwidth];
	void mapCharToValue();
	void stringsToImage(std::string msg);
	void imageToString(int image[3][mwidth][mwidth]);
	void clearBuffer();
};


#endif //IMAGECONVERTER_H
