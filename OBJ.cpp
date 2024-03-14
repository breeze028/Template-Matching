/**************************************************************************************************/
// Template Matching in C++ without image processing libraries. Implement some features like 
// generating grayscale image, drawing a bounding box in image, Gaussian filtering, calculating  
// NCC, downsampling and nearest interpolation.
//
// Created by: Zhuang Su, 2024.2.29-2024.3.13
//
// Need C++20 compilation enviornment(used std::format)
//
/**************************************************************************************************/




// c++ headers
#include <cmath>
#include <complex>
#include <ios>
#include <iostream>
#include <memory>
#include <fstream>
#include <ostream>
#include <vector>
#include <algorithm>
#include <string>
#include <format>
#include <chrono>
#include <assert.h>
#include <stdint.h>
#include <math.h>

// bitmap file loader by Benjamin Kalytta
// http://www.kalytta.com/bitmap.h 

#ifndef __LITTLE_ENDIAN__
	#ifndef __BIG_ENDIAN__
		#define __LITTLE_ENDIAN__
	#endif
#endif

#ifdef __LITTLE_ENDIAN__
	#define BITMAP_SIGNATURE 0x4d42
#else
	#define BITMAP_SIGNATURE 0x424d
#endif

#if defined(_MSC_VER) || defined(__INTEL_COMPILER)
	typedef unsigned __int32 uint32_t;
	typedef unsigned __int16 uint16_t;
	typedef unsigned __int8 uint8_t;
	typedef __int32 int32_t;
#elif defined(__GNUC__) || defined(__CYGWIN__) || defined(__MWERKS__) || defined(__WATCOMC__) || defined(__PGI) || defined(__LCC__)
	#include <stdint.h>
	#include <cstring>
#else
	typedef unsigned int uint32_t;
	typedef unsigned short int uint16_t;
	typedef unsigned char uint8_t;
	typedef int int32_t;
#endif

#pragma pack(push, 1)

typedef struct _BITMAP_FILEHEADER {
	uint16_t Signature;
	uint32_t Size;
	uint32_t Reserved;
	uint32_t BitsOffset;
} BITMAP_FILEHEADER;

#define BITMAP_FILEHEADER_SIZE 14

typedef struct _BITMAP_HEADER {
	uint32_t HeaderSize;
	int32_t Width;
	int32_t Height;
	uint16_t Planes;
	uint16_t BitCount;
	uint32_t Compression;
	uint32_t SizeImage;
	int32_t PelsPerMeterX;
	int32_t PelsPerMeterY;
	uint32_t ClrUsed;
	uint32_t ClrImportant;
	uint32_t RedMask;
	uint32_t GreenMask;
	uint32_t BlueMask;
	uint32_t AlphaMask;
	uint32_t CsType;
	uint32_t Endpoints[9]; // see http://msdn2.microsoft.com/en-us/library/ms536569.aspx
	uint32_t GammaRed;
	uint32_t GammaGreen;
	uint32_t GammaBlue;
} BITMAP_HEADER;

typedef struct _RGBA {
	uint8_t Red;
	uint8_t Green;
	uint8_t Blue;
	uint8_t Alpha;
} RGBA;

typedef struct _BGRA {
	uint8_t Blue;
	uint8_t Green;
	uint8_t Red;
	uint8_t Alpha;
} BGRA;

#pragma pack(pop)

// read and write bitmap files
class CBitmap {
public:
	BITMAP_FILEHEADER m_BitmapFileHeader;
	BITMAP_HEADER m_BitmapHeader;
	RGBA *m_BitmapData;
	unsigned int m_BitmapSize;	
	// Masks and bit counts shouldn't exceed 32 Bits
public:
	class CColor {
public:
		static inline unsigned int BitCountByMask(unsigned int Mask) {
			unsigned int BitCount = 0;
			while (Mask) {
				Mask &= Mask - 1;
				BitCount++;
			}
			return BitCount;
		}

		static inline unsigned int BitPositionByMask(unsigned int Mask) {
			return BitCountByMask((Mask & (~Mask + 1)) - 1);
		}

		static inline unsigned int ComponentByMask(unsigned int Color, unsigned int Mask) {
			unsigned int Component = Color & Mask;
			return Component >> BitPositionByMask(Mask);
		}

		static inline unsigned int BitCountToMask(unsigned int BitCount) {
			return (BitCount == 32) ? 0xFFFFFFFF : (1 << BitCount) - 1;
		}

		static unsigned int Convert(unsigned int Color, unsigned int FromBitCount, unsigned int ToBitCount) {
			if (ToBitCount < FromBitCount) {
				Color >>= (FromBitCount - ToBitCount);
			} else {
				Color <<= (ToBitCount - FromBitCount);
				if (Color > 0) {
					Color |= BitCountToMask(ToBitCount - FromBitCount);
				}
			}
			return Color;
		}
	};

public:
	
	CBitmap() : m_BitmapData(0), m_BitmapSize(0)  {
		Dispose();
	}
	
	CBitmap(const char* Filename) : m_BitmapData(0), m_BitmapSize(0) {
		Load(Filename);
	}
	
	~CBitmap() {
		Dispose();
	}
	
	void Dispose() {
		if (m_BitmapData) {
			delete[] m_BitmapData;
			m_BitmapData = 0;
		}
		memset(&m_BitmapFileHeader, 0, sizeof(m_BitmapFileHeader));
		memset(&m_BitmapHeader, 0, sizeof(m_BitmapHeader));
	}
	
	/* Load specified Bitmap and stores it as RGBA in an internal buffer */
	
	bool Load(const char *Filename) {
		std::ifstream file(Filename, std::ios::binary | std::ios::in);
		
		if (file.bad()) {
			return false;
		}

		if (file.is_open() == false) {
			return false;
		}
		
		Dispose();
		
		file.read((char*) &m_BitmapFileHeader, BITMAP_FILEHEADER_SIZE);
		if (m_BitmapFileHeader.Signature != BITMAP_SIGNATURE) {
			return false;
		}

		file.read((char*) &m_BitmapHeader, sizeof(BITMAP_HEADER));
		
		/* Load Color Table */
		
		file.seekg(BITMAP_FILEHEADER_SIZE + m_BitmapHeader.HeaderSize, std::ios::beg);
		
		unsigned int ColorTableSize = 0;

		if (m_BitmapHeader.BitCount == 1) {
			ColorTableSize = 2;
		} else if (m_BitmapHeader.BitCount == 4) {
			ColorTableSize = 16;
		} else if (m_BitmapHeader.BitCount == 8) {
			ColorTableSize = 256;
		}
		
		// Always allocate full sized color table

		BGRA* ColorTable = new BGRA[ColorTableSize]; // std::bad_alloc exception should be thrown if memory is not available
		
		file.read((char*) ColorTable, sizeof(BGRA) * m_BitmapHeader.ClrUsed);

		/* ... Color Table for 16 bits images are not supported yet */	
		
		m_BitmapSize = GetWidth() * GetHeight();
		m_BitmapData = new RGBA[m_BitmapSize];
		
		unsigned int LineWidth = ((GetWidth() * GetBitCount() / 8) + 3) & ~3;
		uint8_t *Line = new uint8_t[LineWidth];
		
		file.seekg(m_BitmapFileHeader.BitsOffset, std::ios::beg);

		int Index = 0;
		bool Result = true;

		if (m_BitmapHeader.Compression == 0) {
			for (unsigned int i = 0; i < GetHeight(); i++) {
				file.read((char*) Line, LineWidth);

				uint8_t *LinePtr = Line;
				
				for (unsigned int j = 0; j < GetWidth(); j++) {
					if (m_BitmapHeader.BitCount == 1) {
						uint32_t Color = *((uint8_t*) LinePtr);
						for (int k = 0; k < 8; k++) {
							m_BitmapData[Index].Red = ColorTable[Color & 0x80 ? 1 : 0].Red;
							m_BitmapData[Index].Green = ColorTable[Color & 0x80 ? 1 : 0].Green;
							m_BitmapData[Index].Blue = ColorTable[Color & 0x80 ? 1 : 0].Blue;
							m_BitmapData[Index].Alpha = ColorTable[Color & 0x80 ? 1 : 0].Alpha;
							Index++;
							Color <<= 1;
						}
						LinePtr++;
						j += 7;
					} else if (m_BitmapHeader.BitCount == 4) {
						uint32_t Color = *((uint8_t*) LinePtr);
						m_BitmapData[Index].Red = ColorTable[(Color >> 4) & 0x0f].Red;
						m_BitmapData[Index].Green = ColorTable[(Color >> 4) & 0x0f].Green;
						m_BitmapData[Index].Blue = ColorTable[(Color >> 4) & 0x0f].Blue;
						m_BitmapData[Index].Alpha = ColorTable[(Color >> 4) & 0x0f].Alpha;
						Index++;
						m_BitmapData[Index].Red = ColorTable[Color & 0x0f].Red;
						m_BitmapData[Index].Green = ColorTable[Color & 0x0f].Green;
						m_BitmapData[Index].Blue = ColorTable[Color & 0x0f].Blue;
						m_BitmapData[Index].Alpha = ColorTable[Color & 0x0f].Alpha;
						Index++;
						LinePtr++;
						j++;
					} else if (m_BitmapHeader.BitCount == 8) {
						uint32_t Color = *((uint8_t*) LinePtr);
						m_BitmapData[Index].Red = ColorTable[Color].Red;
						m_BitmapData[Index].Green = ColorTable[Color].Green;
						m_BitmapData[Index].Blue = ColorTable[Color].Blue;
						m_BitmapData[Index].Alpha = ColorTable[Color].Alpha;
						Index++;
						LinePtr++;
					} else if (m_BitmapHeader.BitCount == 16) {
						uint32_t Color = *((uint16_t*) LinePtr);
						m_BitmapData[Index].Red = ((Color >> 10) & 0x1f) << 3;
						m_BitmapData[Index].Green = ((Color >> 5) & 0x1f) << 3;
						m_BitmapData[Index].Blue = (Color & 0x1f) << 3;
						m_BitmapData[Index].Alpha = 255;
						Index++;
						LinePtr += 2;
					} else if (m_BitmapHeader.BitCount == 24) {
						uint32_t Color = *((uint32_t*) LinePtr);
						m_BitmapData[Index].Blue = Color & 0xff;
						m_BitmapData[Index].Green = (Color >> 8) & 0xff;
						m_BitmapData[Index].Red = (Color >> 16) & 0xff;
						m_BitmapData[Index].Alpha = 255;
						Index++;
						LinePtr += 3;
					} else if (m_BitmapHeader.BitCount == 32) {
						uint32_t Color = *((uint32_t*) LinePtr);
						m_BitmapData[Index].Blue = Color & 0xff;
						m_BitmapData[Index].Green = (Color >> 8) & 0xff;
						m_BitmapData[Index].Red = (Color >> 16) & 0xff;
						m_BitmapData[Index].Alpha = Color >> 24;
						Index++;
						LinePtr += 4;
					}
				}
			}
		} else if (m_BitmapHeader.Compression == 1) { // RLE 8
			uint8_t Count = 0;
			uint8_t ColorIndex = 0;
			int x = 0, y = 0;

			while (file.eof() == false) {
				file.read((char*) &Count, sizeof(uint8_t));
				file.read((char*) &ColorIndex, sizeof(uint8_t));

				if (Count > 0) {
					Index = x + y * GetWidth();
					for (int k = 0; k < Count; k++) {
						m_BitmapData[Index + k].Red = ColorTable[ColorIndex].Red;
						m_BitmapData[Index + k].Green = ColorTable[ColorIndex].Green;
						m_BitmapData[Index + k].Blue = ColorTable[ColorIndex].Blue;
						m_BitmapData[Index + k].Alpha = ColorTable[ColorIndex].Alpha;
					}
					x += Count;
				} else if (Count == 0) {
					int Flag = ColorIndex;
					if (Flag == 0) {
						x = 0;
						y++;
					} else if (Flag == 1) {
						break;
					} else if (Flag == 2) {
						char rx = 0;
						char ry = 0;
						file.read((char*) &rx, sizeof(char));
						file.read((char*) &ry, sizeof(char));
						x += rx;
						y += ry;
					} else {
						Count = Flag;
						Index = x + y * GetWidth();
						for (int k = 0; k < Count; k++) {
							file.read((char*) &ColorIndex, sizeof(uint8_t));
							m_BitmapData[Index + k].Red = ColorTable[ColorIndex].Red;
							m_BitmapData[Index + k].Green = ColorTable[ColorIndex].Green;
							m_BitmapData[Index + k].Blue = ColorTable[ColorIndex].Blue;
							m_BitmapData[Index + k].Alpha = ColorTable[ColorIndex].Alpha;
						}
						x += Count;
						// Attention: Current Microsoft STL implementation seems to be buggy, tellg() always returns 0.
						if (file.tellg() & 1) {
							file.seekg(1, std::ios::cur);
						}
					}
				}
			}
		} else if (m_BitmapHeader.Compression == 2) { // RLE 4
			/* RLE 4 is not supported */
			Result = false;
		} else if (m_BitmapHeader.Compression == 3) { // BITFIELDS
			
			/* We assumes that mask of each color component can be in any order */

			uint32_t BitCountRed = CColor::BitCountByMask(m_BitmapHeader.RedMask);
			uint32_t BitCountGreen = CColor::BitCountByMask(m_BitmapHeader.GreenMask);
			uint32_t BitCountBlue = CColor::BitCountByMask(m_BitmapHeader.BlueMask);
			uint32_t BitCountAlpha = CColor::BitCountByMask(m_BitmapHeader.AlphaMask);

			for (unsigned int i = 0; i < GetHeight(); i++) {
				file.read((char*) Line, LineWidth);
				
				uint8_t *LinePtr = Line;
				
				for (unsigned int j = 0; j < GetWidth(); j++) {
					
					uint32_t Color = 0;

					if (m_BitmapHeader.BitCount == 16) {
						Color = *((uint16_t*) LinePtr);
						LinePtr += 2;
					} else if (m_BitmapHeader.BitCount == 32) {
						Color = *((uint32_t*) LinePtr);
						LinePtr += 4;
					} else {
						// Other formats are not valid
					}
					m_BitmapData[Index].Red = CColor::Convert(CColor::ComponentByMask(Color, m_BitmapHeader.RedMask), BitCountRed, 8);
					m_BitmapData[Index].Green = CColor::Convert(CColor::ComponentByMask(Color, m_BitmapHeader.GreenMask), BitCountGreen, 8);
					m_BitmapData[Index].Blue = CColor::Convert(CColor::ComponentByMask(Color, m_BitmapHeader.BlueMask), BitCountBlue, 8);
					m_BitmapData[Index].Alpha = CColor::Convert(CColor::ComponentByMask(Color, m_BitmapHeader.AlphaMask), BitCountAlpha, 8);

					Index++;
				}
			}
		}
		
		delete [] ColorTable;
		delete [] Line;

		file.close();
		return Result;
	}
	
	bool Save(const char* Filename, unsigned int BitCount = 32) {
		bool Result = true;

		std::ofstream file(Filename, std::ios::out | std::ios::binary);

		if (file.is_open() == false) {
			return false;
		}
		
		BITMAP_FILEHEADER bfh;
		BITMAP_HEADER bh;
		memset(&bfh, 0, sizeof(bfh));
		memset(&bh, 0, sizeof(bh));

		bfh.Signature = BITMAP_SIGNATURE;
		bfh.BitsOffset = BITMAP_FILEHEADER_SIZE + sizeof(BITMAP_HEADER);
		bfh.Size = (GetWidth() * GetHeight() * BitCount) / 8 + bfh.BitsOffset;
	
		bh.HeaderSize = sizeof(BITMAP_HEADER);
		bh.BitCount = BitCount;
		
		if (BitCount == 32) {
			bh.Compression = 3; // BITFIELD
			bh.AlphaMask = 0xff000000;
			bh.BlueMask = 0x00ff0000;
			bh.GreenMask = 0x0000ff00;
			bh.RedMask = 0x000000ff;
		} else if (BitCount == 16) {
			bh.Compression = 3; // BITFIELD
			bh.AlphaMask = 0x00000000;
			bh.BlueMask = 0x0000001f;
			bh.GreenMask = 0x000007E0;
			bh.RedMask = 0x0000F800;
		} else {
			bh.Compression = 0; // RGB
		}
		
		unsigned int LineWidth = (GetWidth() + 3) & ~3;

		bh.Planes = 1;
		bh.Height = GetHeight();
		bh.Width = GetWidth();
		bh.SizeImage = (LineWidth * BitCount * GetHeight()) / 8;
		bh.PelsPerMeterX = 3780;
		bh.PelsPerMeterY = 3780;
		
		if (BitCount == 32) {
			file.write((char*) &bfh, sizeof(BITMAP_FILEHEADER));
			file.write((char*) &bh, sizeof(BITMAP_HEADER));
			file.write((char*) m_BitmapData, bh.SizeImage);
		} else if (BitCount < 16) {
			uint8_t* Bitmap = new uint8_t[bh.SizeImage];
			
			BGRA *Palette = 0;
			unsigned int PaletteSize = 0;

			if (GetBitsWithPalette(Bitmap, bh.SizeImage, BitCount, Palette, PaletteSize)) {
				bfh.BitsOffset += PaletteSize * sizeof(BGRA);

				file.write((char*) &bfh, BITMAP_FILEHEADER_SIZE);
				file.write((char*) &bh, sizeof(BITMAP_HEADER));
				file.write((char*) Palette, PaletteSize * sizeof(BGRA));
				file.write((char*) Bitmap, bh.SizeImage);
			}
			delete [] Bitmap;
			delete [] Palette;
		} else {
			uint32_t RedMask = 0;
			uint32_t GreenMask = 0;
			uint32_t BlueMask = 0;
			uint32_t AlphaMask = 0;

			if (BitCount == 16) {
				RedMask = 0x0000F800;
				GreenMask = 0x000007E0;
				BlueMask = 0x0000001F;
				AlphaMask = 0x00000000;
			} else if (BitCount == 24) {
				RedMask = 0x00FF0000;
				GreenMask = 0x0000FF00;
				BlueMask = 0x000000FF;
			} else {
				// Other color formats are not valid
				Result = false;
			}

			if (Result) {
				if (GetBits(NULL, bh.SizeImage, RedMask, GreenMask, BlueMask, AlphaMask)) {
					uint8_t* Bitmap = new uint8_t[bh.SizeImage];
					if (GetBits(Bitmap, bh.SizeImage, RedMask, GreenMask, BlueMask, AlphaMask)) {
						file.write((char*) &bfh, sizeof(BITMAP_FILEHEADER));
						file.write((char*) &bh, sizeof(BITMAP_HEADER));
						file.write((char*) Bitmap, bh.SizeImage);
					}
					delete [] Bitmap;
				}
			}
		}

		file.close();
		return Result;
	}

	unsigned int GetWidth() const {
		/* Add plausibility test */
		// if (abs(m_BitmapHeader.Width) > 8192) {
		//	m_BitmapHeader.Width = 8192;
		// }
		return m_BitmapHeader.Width < 0 ? -m_BitmapHeader.Width : m_BitmapHeader.Width;
	}
	
	unsigned int GetHeight() const {
		/* Add plausibility test */
		// if (abs(m_BitmapHeader.Height) > 8192) {
		//	m_BitmapHeader.Height = 8192;
		// }
		return m_BitmapHeader.Height < 0 ? -m_BitmapHeader.Height : m_BitmapHeader.Height;
	}

	unsigned int GetSize() const {
		unsigned int width = m_BitmapHeader.Width < 0 ? -m_BitmapHeader.Width : m_BitmapHeader.Width;
		unsigned int height = m_BitmapHeader.Height < 0 ? -m_BitmapHeader.Height : m_BitmapHeader.Height;
		return width * height;
	}
	
	unsigned int GetBitCount() const {
		/* Add plausibility test */
		// if (m_BitmapHeader.BitCount > 32) {
		//	m_BitmapHeader.BitCount = 32;
		// }
		return m_BitmapHeader.BitCount;
	}
	
	/* Copies internal RGBA buffer to user specified buffer */
	
	bool GetBits(void* Buffer, unsigned int &Size) const {
		bool Result = false;
		if (Size == 0 || Buffer == 0) {
			Size = m_BitmapSize * sizeof(RGBA);
			Result = m_BitmapSize != 0;
		} else {
			memcpy(Buffer, m_BitmapData, Size);
			Result = true;
		}
		return Result;
	}

	/* Returns internal RGBA buffer */
	
	void* GetBits() {
		return m_BitmapData;
	}
	
	/* Copies internal RGBA buffer to user specified buffer and converts it into destination
	 * bit format specified by component masks.
	 *
	 * Typical Bitmap color formats (BGR/BGRA):
	 *
	 * Masks for 16 bit (5-5-5): ALPHA = 0x00000000, RED = 0x00007C00, GREEN = 0x000003E0, BLUE = 0x0000001F
	 * Masks for 16 bit (5-6-5): ALPHA = 0x00000000, RED = 0x0000F800, GREEN = 0x000007E0, BLUE = 0x0000001F
	 * Masks for 24 bit: ALPHA = 0x00000000, RED = 0x00FF0000, GREEN = 0x0000FF00, BLUE = 0x000000FF
	 * Masks for 32 bit: ALPHA = 0xFF000000, RED = 0x00FF0000, GREEN = 0x0000FF00, BLUE = 0x000000FF
	 *
	 * Other color formats (RGB/RGBA):
	 *
	 * Masks for 32 bit (RGBA): ALPHA = 0xFF000000, RED = 0x000000FF, GREEN = 0x0000FF00, BLUE = 0x00FF0000
	 *
	 * Bit count will be rounded to next 8 bit boundary. If IncludePadding is true, it will be ensured
	 * that line width is a multiple of 4. padding bytes are included if necessary.
	 *
	 * NOTE: systems with big endian byte order may require masks in inversion order.
	 */

	bool GetBits(void* Buffer, unsigned int &Size, unsigned int RedMask, unsigned int GreenMask, unsigned int BlueMask, unsigned int AlphaMask, bool IncludePadding = true) {
		bool Result = false;

		uint32_t BitCountRed = CColor::BitCountByMask(RedMask);
		uint32_t BitCountGreen = CColor::BitCountByMask(GreenMask);
		uint32_t BitCountBlue = CColor::BitCountByMask(BlueMask);
		uint32_t BitCountAlpha = CColor::BitCountByMask(AlphaMask);
		
		unsigned int BitCount = (BitCountRed + BitCountGreen + BitCountBlue + BitCountAlpha + 7) & ~7;

		if (BitCount > 32) {
			return false;
		}
		
		unsigned int w = GetWidth();
		//unsigned int LineWidth = (w + 3) & ~3;
		unsigned int dataBytesPerLine = (w * BitCount + 7) / 8;
		unsigned int LineWidth = (dataBytesPerLine + 3) & ~3;

		if (Size == 0 || Buffer == 0) {
			//Size = (LineWidth * GetHeight() * BitCount) / 8 + sizeof(unsigned int);
			Size = (GetWidth() * GetHeight() * BitCount) / 8 + sizeof(unsigned int);
			return true;
		}

		uint8_t* BufferPtr = (uint8_t*) Buffer;
		
		Result = true;

		uint32_t BitPosRed = CColor::BitPositionByMask(RedMask);
		uint32_t BitPosGreen = CColor::BitPositionByMask(GreenMask);
		uint32_t BitPosBlue = CColor::BitPositionByMask(BlueMask);
		uint32_t BitPosAlpha = CColor::BitPositionByMask(AlphaMask);
		
		unsigned int j = 0;

		for (unsigned int i = 0; i < m_BitmapSize; i++) {
			*(uint32_t*) BufferPtr =
			(CColor::Convert(m_BitmapData[i].Blue, 8, BitCountBlue) << BitPosBlue) |
			(CColor::Convert(m_BitmapData[i].Green, 8, BitCountGreen) << BitPosGreen) | 
			(CColor::Convert(m_BitmapData[i].Red, 8, BitCountRed) << BitPosRed) | 
			(CColor::Convert(m_BitmapData[i].Alpha, 8, BitCountAlpha) << BitPosAlpha);
			
			if (IncludePadding) {
				j++;
				if (j >= w) {
					for (unsigned int k = 0; k < LineWidth - dataBytesPerLine; k++) {
						BufferPtr += (BitCount >> 3);
					}
					j = 0;
				}
			}

			BufferPtr += (BitCount >> 3);
		}
		
		Size -= sizeof(unsigned int);

		return Result;
	}
	
	/* See GetBits(). 
	 * It creates a corresponding color table (palette) which have to be destroyed by the user after usage.
	 *
	 * Supported Bit depths are: 4, 8
	 *
	 * Todo: Optimize, use optimized palette, do ditehring (see my dithering class), support padding for 4 bit bitmaps
	 */

	bool GetBitsWithPalette(void* Buffer, unsigned int &Size, unsigned int BitCount, BGRA* &Palette, unsigned int &PaletteSize, bool OptimalPalette = false, bool IncludePadding = true) {
		bool Result = false;

		if (BitCount > 16) {
			return false;
		}
		
		unsigned int w = GetWidth();
		unsigned int dataBytesPerLine = (w * BitCount + 7) / 8;
		unsigned int LineWidth = (dataBytesPerLine + 3) & ~3;

		if (Size == 0 || Buffer == 0) {
			Size = (LineWidth * GetHeight() * BitCount) / 8;
			return true;
		}
		

		if (OptimalPalette) {
			PaletteSize = 0;
			// Not implemented
		} else {
			if (BitCount == 1) {
				PaletteSize = 2;
				// Not implemented: Who need that?
			} else if (BitCount == 4) { // 2:2:1
				PaletteSize = 16;
				Palette = new BGRA[PaletteSize];
				for (int r = 0; r < 4; r++) {
					for (int g = 0; g < 2; g++) {
						for (int b = 0; b < 2; b++) {
							Palette[r | g << 2 | b << 3].Red = r ? (r << 6) | 0x3f : 0;
							Palette[r | g << 2 | b << 3].Green = g ? (g << 7) | 0x7f : 0;
							Palette[r | g << 2 | b << 3].Blue = b ? (b << 7) | 0x7f : 0;
							Palette[r | g << 2 | b << 3].Alpha = 0xff;
						}
					}
				}
			} else if (BitCount == 8) { // 3:3:2
				PaletteSize = 256;
				Palette = new BGRA[PaletteSize];
				for (int r = 0; r < 8; r++) {
					for (int g = 0; g < 8; g++) {
						for (int b = 0; b < 4; b++) {
							Palette[r | g << 3 | b << 6].Red = r ? (r << 5) | 0x1f : 0;
							Palette[r | g << 3 | b << 6].Green = g ? (g << 5) | 0x1f : 0;
							Palette[r | g << 3 | b << 6].Blue = b ? (b << 6) | 0x3f : 0;
							Palette[r | g << 3 | b << 6].Alpha = 0xff;
						}
					}
				}
			} else if (BitCount == 16) { // 5:5:5
				// Not implemented
			}
		}

		unsigned int j = 0;
		uint8_t* BufferPtr = (uint8_t*) Buffer;
		
		for (unsigned int i = 0; i < m_BitmapSize; i++) {
			if (BitCount == 1) {
				// Not implemented: Who needs that?
			} else if (BitCount == 4) {
				*BufferPtr = ((m_BitmapData[i].Red >> 6) | (m_BitmapData[i].Green >> 7) << 2 | (m_BitmapData[i].Blue >> 7) << 3) << 4;
				i++;
				*BufferPtr |= (m_BitmapData[i].Red >> 6) | (m_BitmapData[i].Green >> 7) << 2 | (m_BitmapData[i].Blue >> 7) << 3;
			} else if (BitCount == 8) {
				*BufferPtr = (m_BitmapData[i].Red >> 5) | (m_BitmapData[i].Green >> 5) << 3 | (m_BitmapData[i].Blue >> 5) << 6;
			} else if (BitCount == 16) {
				// Not implemented
			}

			if (IncludePadding) {
				j++;
				if (j >= w) {
					for (unsigned int k = 0; k < (LineWidth - dataBytesPerLine); k++) {
						BufferPtr += BitCount / 8;
					}
					j = 0;
				}
			}

			BufferPtr++;
		}
		
		Result = true;

		return Result;
	}

	/* Set Bitmap Bits. Will be converted to RGBA internally */

	bool SetBits(void* Buffer, unsigned int Width, unsigned int Height, unsigned int RedMask, unsigned int GreenMask, unsigned int BlueMask, unsigned int AlphaMask = 0) {
		if (Buffer == 0) {
			return false;
		}

		uint8_t *BufferPtr = (uint8_t*) Buffer;
		
		Dispose();

		m_BitmapHeader.Width = Width;
		m_BitmapHeader.Height = Height;
		m_BitmapHeader.BitCount = 32;
		m_BitmapHeader.Compression = 3; 

		m_BitmapSize = GetWidth() * GetHeight();
		m_BitmapData = new RGBA[m_BitmapSize];
		
		/* Find bit count by masks (rounded to next 8 bit boundary) */
		
		unsigned int BitCount = (CColor::BitCountByMask(RedMask | GreenMask | BlueMask | AlphaMask) + 7) & ~7;
		
		uint32_t BitCountRed = CColor::BitCountByMask(RedMask);
		uint32_t BitCountGreen = CColor::BitCountByMask(GreenMask);
		uint32_t BitCountBlue = CColor::BitCountByMask(BlueMask);
		uint32_t BitCountAlpha = CColor::BitCountByMask(AlphaMask);

		for (unsigned int i = 0; i < m_BitmapSize; i++) {
			unsigned int Color = 0;
			if (BitCount <= 8) {
				Color = *((uint8_t*) BufferPtr);
				BufferPtr += 1;
			} else if (BitCount <= 16) {
				Color = *((uint16_t*) BufferPtr);
				BufferPtr += 2;
			} else if (BitCount <= 24) {
				Color = *((uint32_t*) BufferPtr);
				BufferPtr += 3;
			} else if (BitCount <= 32) {
				Color = *((uint32_t*) BufferPtr);
				BufferPtr += 4;
			} else {
				/* unsupported */
				BufferPtr += 1;
			}
			m_BitmapData[i].Alpha = CColor::Convert(CColor::ComponentByMask(Color, AlphaMask), BitCountAlpha, 8);
			m_BitmapData[i].Red = CColor::Convert(CColor::ComponentByMask(Color, RedMask), BitCountRed, 8);
			m_BitmapData[i].Green = CColor::Convert(CColor::ComponentByMask(Color, GreenMask), BitCountGreen, 8);
			m_BitmapData[i].Blue = CColor::Convert(CColor::ComponentByMask(Color, BlueMask), BitCountBlue, 8);
		}

		return true;
	}
};

















//****************************************************************************************************************//
















// a simple matrix class to denote a pixel's coordinate (x, y)
template <class T>
class matrix
{
public:
    matrix(unsigned int rows, unsigned int cols) : rows(rows), cols(cols)
    {
        allocSpace();
    }

    // correspond to the coordinate system
    matrix(uint8_t* data, unsigned int rows, unsigned int cols) : rows(rows), cols(cols)
    {
        allocSpace();
        for (unsigned int i = 0; i < rows; ++i)
        {
            for (unsigned int j = 0; j < cols; ++j)
            {
                p[i][j] = data[(rows - i - 1) * cols + j];
            }
        }
    }

    ~matrix()
    {
        for (unsigned int i = 0; i < rows; ++i)
        {
            delete[] p[i];
        }
        delete[] p;
    }

    T* operator[](unsigned int x)
    {
        return p[x];
    }

private:
    inline void allocSpace()
    {
        p = new T* [rows];
        for (unsigned int i = 0; i < rows; ++i)
        {
            p[i] = new T[cols];
        }
    }

    unsigned int rows;
    unsigned int cols;
    T** p;

};















//****************************************************************************************************************//




















// output format for this project
struct OUTPUTFORMAT
{
	unsigned int x;
	unsigned int y;
	float accuracy;
	float IoU;

	// not for output
	unsigned int templ_scaled_width;
	unsigned int templ_scaled_height;
};

// function declaration
uint8_t R8G8B8A82GR(RGBA rgba);
void DrawRectangle(unsigned int x, unsigned int y, unsigned int width, unsigned int height);
void GenerateGaryscaleImage(CBitmap* bmp, uint8_t* gray_buffer);
void GaussianFilter(CBitmap* bmp);
void GaussianFilterNTimes(CBitmap* bmp, unsigned int times);
void DownSample(CBitmap* bmp);                           // half the width and height, this function is now useless.
void DownSampleNTimes(CBitmap* bmp, unsigned int times); // same as above
void NearestScaling(CBitmap* bmp, float scaleWidth, float scaleHeight);
bool DescendingWithAccuracy(OUTPUTFORMAT a, OUTPUTFORMAT b);
int Clamp(int x, int min, int max);
void TemplateMatching(int num, bool save = false);

// global varibles
static std::unique_ptr<CBitmap> image_bmp;
static std::unique_ptr<CBitmap> templ_bmp;
static std::string image_name;
static std::string templ_name;

// ground truth
struct coordinates
{
	unsigned int x;
	unsigned int y;
};

// match manually
static coordinates ground_truth[101] = 
{{537, 420}, {782, 392}, {562, 375}, {1000, 232}, {456, 396},
 {331, 388}, {1070, 390}, {432, 362}, {992, 440}, {451, 408},
{1, 289}, {1424, 375}, {1281, 398}, {558, 317}, {1413, 400},
{486, 331}, {773, 333}, {320, 358}, {538, 368}, {353, 315},
{1118, 361}, {317, 586}, {363, 576}, {4, 598}, {196, 420},
{513, 348}, {1378, 409}, {430, 526}, {935, 319}, {789, 411},
{1175, 385}, {653, 410}, {642, 364}, {845, 330}, {1216, 349},
{350, 414}, {134, 340}, {362, 347}, {1127, 331}, {1138, 470},
{666, 364}, {242, 223}, {612, 443}, {208, 381}, {1313, 333},
{1088, 369}, {321, 270}, {1640, 391},{914, 364}, {3, 363},
{1092, 238}, {794, 398}, {20, 355}, {1071, 311}, {118, 345},
{1076, 219},{540, 288}, {639, 379}, {829, 262}, {56, 293},
{1092, 370}, {586, 463}, {857, 408}, {1113, 461}, {940, 481},
{191, 263}, {530, 359}, {1127, 327}, {1140, 355}, {420, 465},
{395, 464}, {719, 1}, {733, 651}, {946, 495}, {571, 434},
{738, 293}, {980, 145}, {797, 591}, {855, 1}, {357, 263},
{718, 276}, {244, 644}, {352, 273}, {1069, 548}, {473, 461},
{66, 396}, {550, 160}, {198, 371}, {403, 374}, {428, 400},
{672, 377}, {1038, 285}, {23, 344}, {8, 370}, {827, 368},
{1050, 250}, {1122, 358}, {526, 380}, {692, 274}, {927, 214},
{155, 317}};



// the main function
int main(int argc, char** argv){

	// only for debug, you can just ignore it.
	// output the basic .txt and a source image with bounding boxes.  
	// some images cannot be output,(such as test002.bmp and test003.bmp), which is caused by the Save(const char*)  
	// function in CBitmap class. And I am not going to fix it.
	// e.g.  Terminal:
	// D:pj1\build> .\pj1.exe 1
	// it will automatically search for the test001.bmp and obj001.bmp, then output debug information.
	if(argc == 2)
	{
		int id = std::stoi(argv[1]);
		if(id < 1 || id > 100) return 0;

		image_name = std::format("test{:03}.bmp", id);
		templ_name = std::format("obj{:03}.bmp", id);

		std::clog << "\rimages remaining: " << 1 << ' ' << std::flush;
		TemplateMatching(id, true);

		return 0;
	}

	// match for input1.bmp
	image_name = "input1.bmp";
	templ_name = "input2.bmp";

	std::clog << "\rimages remaining: " << 1 << ' ' << std::flush;
	TemplateMatching(0);


	//match for test001.bmp ~ test100.bmp
	
	// for(int m = 1; m < 101; ++m)
	// {
	// 	std::clog << "\rimages remaining: " << 101 - m << ' ' << std::flush;
	// 	image_name = std::format("test{:03}.bmp", m);
	// 	templ_name = std::format("obj{:03}.bmp", m);

	// 	TemplateMatching(m);	
	// }

	return 0;
	
}


uint8_t R8G8B8A82GR(RGBA rgba)
{
	// quick calculation  
    return (rgba.Red * 76 + rgba.Green * 150 + rgba.Blue * 30) >> 8;
}

void DrawRectangle(unsigned int x, unsigned int y, unsigned int width, unsigned int height)
{
	// cache
	unsigned int image_width = image_bmp->GetWidth();
	unsigned int image_height = image_bmp->GetHeight();

	// Y-axis
	for(unsigned int i = y; i < y + height; ++i)
    {
        image_bmp->m_BitmapData[(image_height - i - 1) * image_width + x].Red = 0;
        image_bmp->m_BitmapData[(image_height - i - 1) * image_width + x].Green = 255;
        image_bmp->m_BitmapData[(image_height - i - 1) * image_width + x].Blue = 0;

        image_bmp->m_BitmapData[(image_height - i - 1) * image_width + x + width - 1].Red = 0;
        image_bmp->m_BitmapData[(image_height - i - 1) * image_width + x + width - 1].Green = 255;
        image_bmp->m_BitmapData[(image_height - i - 1) * image_width + x + width - 1].Blue = 0;
    }

	// X-axis
	for(unsigned int i = x; i < x + width; ++i)
    {
        image_bmp->m_BitmapData[(image_height - y - 1) * image_width + i].Red = 0;
        image_bmp->m_BitmapData[(image_height - y - 1) * image_width + i].Green = 255;
        image_bmp->m_BitmapData[(image_height - y - 1) * image_width + i].Blue = 0;

        image_bmp->m_BitmapData[(image_height - y - height) * image_width + i].Red = 0;
        image_bmp->m_BitmapData[(image_height - y - height) * image_width + i].Green = 255;
        image_bmp->m_BitmapData[(image_height - y - height) * image_width + i].Blue = 0;
    }

}

void GenerateGaryscaleImage(CBitmap* bmp, uint8_t* gray_buffer)
{
	for(unsigned int i = 0; i < bmp->GetSize(); ++i)
    {
        bmp->m_BitmapData[i].Red = gray_buffer[i];
        bmp->m_BitmapData[i].Green = gray_buffer[i];
        bmp->m_BitmapData[i].Blue = gray_buffer[i];
    }

}


// change the m_BitmapData property. should call this function before get gray buffer
void GaussianFilter(CBitmap* bmp)
{
	// one dimension gaussian kernel
	float weight[] = {0.4026f, 0.2442f, 0.0545f};

	// instantiate the matrix
	matrix<RGBA> pixels(bmp->GetHeight() + 4, bmp->GetWidth() + 4);

	for(unsigned int i = 2; i < bmp->GetHeight() + 2; ++i)
	{
		for(unsigned int j = 2; j < bmp->GetWidth() + 2; ++j)
		{
			unsigned int index = (bmp->GetHeight() - i + 1) * bmp->GetWidth() + j - 2;
			pixels[i][j] = bmp->m_BitmapData[index];			
		}
	}

	// convonlution
	// X-axis
	for(unsigned int i = 2; i < bmp->GetHeight() + 2; ++i)
	{
		for(unsigned int j = 2; j < bmp->GetWidth() + 2; ++j)
		{
			float sumR = pixels[i][j].Red * weight[0];
			float sumG = pixels[i][j].Green * weight[0];
			float sumB = pixels[i][j].Blue * weight[0];

			for(unsigned int m = 1; m < 3; ++m)
			{
				sumR += pixels[i][j + m].Red * weight[m];
				sumG += pixels[i][j + m].Green * weight[m];
				sumB += pixels[i][j + m].Blue * weight[m];

				sumR += pixels[i][j - m].Red * weight[m];
				sumG += pixels[i][j - m].Green * weight[m];
				sumB += pixels[i][j - m].Blue * weight[m];
			}

			pixels[i][j].Red = sumR;
			pixels[i][j].Green = sumG;
			pixels[i][j].Blue = sumB;
		}
	}
	// Y-axis
	for(unsigned int i = 2; i < bmp->GetHeight() + 2; ++i)
	{
		for(unsigned int j = 2; j < bmp->GetWidth() + 2; ++j)
		{
			float sumR = pixels[i][j].Red * weight[0];
			float sumG = pixels[i][j].Green * weight[0];
			float sumB = pixels[i][j].Blue * weight[0];

			for(unsigned int m = 1; m < 3; ++m)
			{
				sumR += pixels[i + m][j].Red * weight[m];
				sumG += pixels[i + m][j].Green * weight[m];
				sumB += pixels[i + m][j].Blue * weight[m];

				sumR += pixels[i - m][j].Red * weight[m];
				sumG += pixels[i - m][j].Green * weight[m];
				sumB += pixels[i - m][j].Blue * weight[m];
			}

			pixels[i][j].Red = sumR;
			pixels[i][j].Green = sumG;
			pixels[i][j].Blue = sumB;
		}
	}

	// write back to the bitmap
	for(unsigned int i = 2; i < bmp->GetHeight() + 2; ++i)
	{
		for(unsigned int j = 2; j < bmp->GetWidth() + 2; ++j)
		{
			unsigned int index = (bmp->GetHeight() - i + 1) * bmp->GetWidth() + j - 2;
			bmp->m_BitmapData[index] = pixels[i][j];		
		}
	}

}

void GaussianFilterNTimes(CBitmap* bmp, unsigned int times)
{
	for(int i = 0; i < times; ++i)
	{
		GaussianFilter(bmp);
	}
}


void DownSample(CBitmap* bmp)
{
	unsigned int width = bmp->GetWidth();
	unsigned int height = bmp->GetHeight();
	
	// initialize a matrix
	matrix<RGBA> pixels(height, width);

	for(unsigned int i = 0; i < height; ++i)
	{
		for(unsigned int j = 0; j < width; ++j)
		{
			pixels[i][j] = bmp->m_BitmapData[(height - i - 1) * width + j];
		}
	}

	unsigned int index = 0;
	for(unsigned int i = 0; i < height; i += 2)
	{
		for(unsigned int j = 0; j < width; j += 2)
		{
			bmp->m_BitmapData[index] = pixels[height - i - 1][j];
			index++;
		}
	}

	bmp->m_BitmapHeader.Width = width % 2 ? width / 2 + 1 : width / 2;
	bmp->m_BitmapHeader.Height = height % 2 ? height / 2 + 1 : height / 2;
	bmp->m_BitmapSize = bmp->GetWidth() * bmp->GetHeight();
}

void DownSampleNTimes(CBitmap* bmp, unsigned int times)
{
	for(int i = 0; i < times; ++i)
	{
		DownSample(bmp);
	}
}

int Clamp(int x, int min, int max)
{
	if(x < min) return min;
	if(x > max) return max;
	return x;
}

// only support downscaling
void NearestScaling(CBitmap* bmp, float scaleWidth, float scaleHeight)
{
	if(scaleWidth > 1 || scaleHeight > 1) return;

	unsigned int src_width = bmp->GetWidth();
	unsigned int src_height = bmp->GetHeight();

	auto dst_width = static_cast<unsigned int>(src_width * scaleWidth);
	auto dst_height = static_cast<unsigned int>(src_height * scaleHeight);

	matrix<RGBA> src_pixels(src_height, src_width);

	for(unsigned int i = 0; i < src_height; ++i)
	{
		for(unsigned int j = 0; j < src_width; ++j)
		{
			src_pixels[i][j] = bmp->m_BitmapData[(src_height - i - 1) * src_width + j];
		}
	}

	matrix<RGBA> dst_pixels(dst_height, dst_width);

	for(unsigned int i = 0; i < dst_height; ++i)
	{
		for(unsigned int j = 0; j < dst_width; ++j)
		{
			auto src_i = Clamp(static_cast<unsigned int>(i / scaleHeight), 0, src_height);
			auto src_j = Clamp(static_cast<unsigned int>(j / scaleWidth), 0, src_width);

			dst_pixels[i][j] = src_pixels[src_i][src_j];
		}
	}
 
	unsigned int index = 0;
	for(unsigned int i = 0; i < dst_height; i ++)
	{
		for(unsigned int j = 0; j < dst_width; j ++)
		{
			bmp->m_BitmapData[index] = dst_pixels[dst_height - i - 1][j];
			index++;
		}
	}

	bmp->m_BitmapHeader.Width = dst_width;
	bmp->m_BitmapHeader.Height = dst_height;
	bmp->m_BitmapSize = bmp->GetSize();
	//std::cout << dst_width << ' ' << dst_height << '\n';

}

bool DescendingWithAccuracy(OUTPUTFORMAT a, OUTPUTFORMAT b)
{
	return a.accuracy > b.accuracy;
}


void TemplateMatching(int num, bool save)
{
	// timer
	auto stamp_begin = std::chrono::steady_clock::now();

	// read .bmp files
	assert(image_bmp = std::make_unique<CBitmap>(image_name.c_str()));
    assert(templ_bmp = std::make_unique<CBitmap>(templ_name.c_str()));

	std::vector<OUTPUTFORMAT> res;

	bool flag = false;
	for(float scaleWidth = 0.150f; scaleWidth >= 0.05f; scaleWidth -= 0.050f)
	{
		for(float scaleHeight = 0.05f; scaleHeight <= 0.150f; scaleHeight += 0.050f)
		{
			res.clear();
			std::unique_ptr<CBitmap> image_bmp_copy;
			assert(image_bmp_copy = std::make_unique<CBitmap>(image_name.c_str()));
			GaussianFilterNTimes(image_bmp_copy.get(), 3);
			NearestScaling(image_bmp_copy.get(), scaleWidth, scaleHeight);

    		// get matrix of pixels in Grayscale
    		// coordinate system: the top left corner is (0, 0), the X-axis points to the right and the Y-axis
    		// downwards. just like DirectX and Photoshop.
    		uint8_t* image_gray_buffer = new uint8_t[image_bmp_copy->GetSize()];
    		RGBA* image_rgba_buffer = reinterpret_cast<RGBA*>(image_bmp_copy->GetBits());

    		for (unsigned int i = 0; i < image_bmp_copy->GetSize(); ++i)
       			image_gray_buffer[i] = R8G8B8A82GR(image_rgba_buffer[i]);

    		matrix<uint8_t> image_gray_pixels(image_gray_buffer, image_bmp_copy->GetHeight(), image_bmp_copy->GetWidth());

    		uint8_t* templ_gray_buffer = new uint8_t[templ_bmp->GetSize()];
    		RGBA* templ_rgba_buffer = reinterpret_cast<RGBA*>(templ_bmp->GetBits());

    		for (unsigned int i = 0; i < templ_bmp->GetSize(); ++i)
       			templ_gray_buffer[i] = R8G8B8A82GR(templ_rgba_buffer[i]);

    		matrix<uint8_t> templ_gray_pixels(templ_gray_buffer, templ_bmp->GetHeight(), templ_bmp->GetWidth());

    		// template matching
    		unsigned int rows = image_bmp_copy->GetHeight() - templ_bmp->GetHeight() + 1;
    		unsigned int cols = image_bmp_copy->GetWidth() - templ_bmp->GetWidth() + 1;

    		matrix<float> ncc(rows, cols);
    		for(unsigned int i = 0; i < rows; ++i)
    		{
        		for(unsigned int j = 0; j < cols; ++j)
        		{
            		// the normalized cross correlation coefficient(NCC)
            		// more information on math:https://blog.csdn.net/fb_help/article/details/104162770
            		// i for image, t for template
            		unsigned int i_sum = 0;
            		unsigned int t_sum = 0;          
            		for(unsigned int m = i; m < i + templ_bmp->GetHeight(); ++m)
            		{
                		for(unsigned int n = j; n < j + templ_bmp->GetWidth(); ++n)
                		{
                    		i_sum += image_gray_pixels[m][n];
                    		t_sum += templ_gray_pixels[m - i][n - j];
                		}
            		}
            		float i_mean = (float)i_sum / templ_bmp->GetSize();
            		float t_mean = (float)t_sum / templ_bmp->GetSize();

            		float i_standard_deviation = 0;
            		float t_standard_deviation = 0;
            		for(unsigned int m = i; m < i + templ_bmp->GetHeight(); ++m)
            		{
                		for(unsigned int n = j; n < j + templ_bmp->GetWidth(); ++n)
                		{
                    		i_standard_deviation += std::pow(image_gray_pixels[m][n] - i_mean, 2);
                    		t_standard_deviation += std::pow(templ_gray_pixels[m - i][n - j] - t_mean, 2);
                		}
            		}
            		i_standard_deviation = std::sqrt(i_standard_deviation / (float) templ_bmp->GetSize());
            		t_standard_deviation = std::sqrt(t_standard_deviation / (float) templ_bmp->GetSize());

            		float num1 = 1.0f / (i_standard_deviation * t_standard_deviation * templ_bmp->GetSize());
            		float r = 0;
            		for(unsigned int m = i; m < i + templ_bmp->GetHeight(); ++m)
            		{
                		for(unsigned int n = j; n < j + templ_bmp->GetWidth(); ++n)
                		{
                 			r += (image_gray_pixels[m][n] - i_mean) * (templ_gray_pixels[m - i][n - j] - t_mean);            
                		}
            		}
            		r *= num1;

            		ncc[i][j] = r;
        		}      
        
    		}

			// store results
			auto templ_scaled_width = static_cast<unsigned int>(templ_bmp->GetWidth() / scaleWidth);
			auto templ_scaled_height = static_cast<unsigned int>(templ_bmp->GetHeight() / scaleHeight);

			for(unsigned int i = 0; i < rows; ++i)
			{
				for(unsigned int j = 0; j < cols; ++j)
				{
					if(ncc[i][j] > 0.6f)
					{
						auto src_i = Clamp(static_cast<unsigned int>(i / scaleHeight), 0, image_bmp->GetHeight());
			    		auto src_j = Clamp(static_cast<unsigned int>(j / scaleWidth), 0, image_bmp->GetWidth());
						
						auto S = templ_scaled_width * templ_scaled_height;
						unsigned int I = 0;
						if(std::abs(src_j - (int)ground_truth[num].x) >= templ_scaled_width || std::abs(src_i - (int)ground_truth[num].y) >= templ_scaled_height) 
							continue;
						else
				 			I = (templ_scaled_width - std::abs(src_j - (int)ground_truth[num].x)) * ( templ_scaled_height - std::abs(src_i - (int)ground_truth[num].y));
					
						OUTPUTFORMAT output;
						output.x = src_j;
						output.y = src_i;
						output.accuracy = (float) I / S;
						output.IoU = (float) I / (2 * S - I);
						output.templ_scaled_width = templ_scaled_width;
						output.templ_scaled_height = templ_scaled_height;

						res.push_back(output);
				
					}
				
				}
			}

			delete[] image_gray_buffer; image_gray_buffer = nullptr;
	        delete[] templ_gray_buffer;	templ_gray_buffer = nullptr;

			std::sort(res.begin(), res.end(), DescendingWithAccuracy);
			if(res.size() > 0 && res[0].accuracy >= 0.8f)
			{
				flag = true;
				break;
			}
		}

		if(flag)
			break;		
	}

	auto stamp_end = std::chrono::steady_clock::now();
	
	std::ofstream file("output.txt", std::ios::app);
	file << image_name << ":\n";
	file << "coordinates accuracy IoU\n";

	if(res.size() > 5)
		res.resize(5);

	float sum = 0.0f;

	for(auto& output : res)
	{
		file << '(' << output.x << ", " << output.y << ") " << output.accuracy << ' ' << output.IoU << '\n'; 
		sum += output.accuracy;

		if(save)
			DrawRectangle(output.x, output.y, output.templ_scaled_width, output.templ_scaled_height);
	}
	file << "average precision:" << (float) sum / res.size() << " " << "processing time(ms):" << 
		std::chrono::duration<double, std::milli>(stamp_end - stamp_begin).count() << "\n\n";
	file.close();

	if(save)
	{
		std::string output_name = "output_" + image_name;
		image_bmp->Save(output_name.c_str());
	}
		
}