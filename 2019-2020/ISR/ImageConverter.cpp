//
// Created by root on 2/9/18.
//
#include "ImageConverter.h"
#include <iostream>
#include <time.h>
#include <map>

ImageConverter::ImageConverter()
{ }

void ImageConverter::mapCharToValue()
{
	bool conv = true;
	int count = 0;
	int tcount = 0;
	int setLoc = 0;
	int lktLoc = 0;
	while(conv)
	{
		if(count == 4)
		{
			++setLoc;
			count = 0;
		}
		intToCharMap[lktLoc] = charSet[setLoc];
		++lktLoc;
		++count;
		if(tcount == maxColorValue)
			conv = false;
		++tcount;
	}

	for (int i = 0; i < 256; ++i)
	{
		charmap.insert(std::pair<char, int>(intToCharMap[i], i));
		i += 3;
	}
}

void ImageConverter::stringsToImage(std::string msg)
{
	char tempchar = ' ';
	char pasttempchar = ' ';
	std::string tempstring = "";
	int row = 0;
	int spec = 0;
	int value = 0;
	for (int i = 0; i < msg.length(); ++i)
	{
		tempstring = "";
		tempchar = msg[i];
		if (tempchar == '|')
		{
			i++;
			tempchar = msg[i];
			while(tempchar != '|')
			{
				tempstring += tempchar;
				i++;
				tempchar = msg[i];
			}
			i++;
			std::string token = tempstring.substr(0,1);
			std::string token2  = tempstring.substr(1,tempstring.size());
			spec = std::atoi(token.c_str());
			row = std::atoi(token2.c_str());

		}
		tempchar = msg[i];
		if(tempchar < '0' || tempchar > '9')
		{
			imageMatrix[spec][col[spec][row]][row] = charmap.find(tempchar)->second;
			col[spec][row]++;
			pasttempchar = tempchar;
		}
		else
		{
			i++;
			tempstring = "";
			tempstring += tempchar;
			tempchar = msg[i];
			while(tempchar >= '0' && tempchar <= '9' )
			{
				tempstring += tempchar;
				i++;
			}
			value = std::atoi(tempstring.c_str());
			for (int j = 0; j < value-1; ++j)
			{
				imageMatrix[spec][col[spec][row]][row] = charmap.find(pasttempchar)->second;
				col[spec][row]++;
			}
			i--;
		}

	}

}


void ImageConverter::imageToString(int image[3][mwidth][mwidth])
{

	// Holds one portion of the image up to 210 characters
	std::string msg;

	// Number of repeating pixel values
	int compressionCount;

	// Pointer as we traverse the image
	char currentChar;

	// Pointer as we traverse the image
	char nextChar;

	// Temporary string for checking message length before committing
	std::string tempMsg = "";

	// For each channel(RGB)
	for(int rgb = 0; rgb < 3; rgb++){

		compressionCount = 0;

		//For each row
		for(int row = 0; row < mwidth; row++){
			compressionCount = 0;

			// Denote spectrum and row value at the beginning of each row
			tempMsg = '|' + std::to_string(rgb) + std::to_string(row) + '|'; // "|[rgb][0-99]|"

			if( msg.size() + tempMsg.size() > maxstring-2){ //If adding to message exceeds limit
				//TODO push out message
				stringBuffer[size_stringBuffer++] = msg;
				msg = "";

			}
			msg += tempMsg;
			tempMsg = "";
			// For each column
			for(int column = 0; column < mwidth; column++){
				//get a char for each pixel value( [0-3] -> a )
				currentChar = intToCharMap[image[rgb][column][row]];

				if(column == 99){ // If at end of row
					nextChar = '/';
				}
				else{
					nextChar = intToCharMap[image[rgb][column+1][row]];
				}

				// If another repeated character
				if(currentChar == nextChar){
					compressionCount++; // Increment repeated count
				}
				else{
					compressionCount++; // Current char is the same so add to repeated count
					tempMsg += currentChar; // Push the current char to msg string
					tempMsg += std::to_string(compressionCount); // Append the count of that character
					if(tempMsg.size() + msg.size() > maxstring){ //If adding to the msg will exceed limit
						//TODO push out message
						stringBuffer[size_stringBuffer++] = msg;
						msg = '|' + std::to_string(rgb) + std::to_string(row) + '|'; // "|[rgb][0-99]|"
					}
					msg += tempMsg;
					tempMsg = "";
					compressionCount = 0; // Reset count
				}

			} // end for imageLength

		}// end for imageWidth

	} // end for rgb
	//TODO push out message
	stringBuffer[size_stringBuffer++] = msg;
} // end imageToString


void ImageConverter::clearBuffer()
{
    for (int i = 0; i < 300; ++i)
    {
        stringBuffer[i] = "";
    }
    size_stringBuffer = 0;
}