// Fill out your copyright notice in the Description page of Project Settings.

#include "PacketBuffer.h"

PacketBuffer::PacketBuffer(const uint32 Width, const uint32 Height, const float FieldOfView) :
  IsDataReadable(false), SizeHeader(sizeof(PacketHeader)), SizeRGB(Width *Height * 3 * sizeof(uint8)), SizeFloat(Width *Height *sizeof(FFloat16)),
  OffsetColor(SizeHeader), OffsetDepth(OffsetColor + SizeRGB), OffsetObject(OffsetDepth + SizeFloat), OffsetMap(OffsetObject + SizeRGB),
  Size(SizeHeader + SizeRGB + SizeFloat + SizeRGB)
{
  ReadBuffer.resize(Size + 1024 * 1024);
  WriteBuffer.resize(Size + 1024 * 1024);

  // Create relative FOV for each axis
  const float FOVX = Height > Width ? FieldOfView * Width / Height : FieldOfView;
  const float FOVY = Width > Height ? FieldOfView * Height / Width : FieldOfView;

  // Setting header information that do not change
  HeaderRead = reinterpret_cast<PacketHeader *>(&ReadBuffer[0]);
  HeaderRead->Size = Size;
  HeaderRead->SizeHeader = SizeHeader;
  HeaderRead->Width = Width;
  HeaderRead->Height = Height;
  HeaderRead->FieldOfViewX = FOVX;
  HeaderRead->FieldOfViewY = FOVY;
  HeaderWrite = reinterpret_cast<PacketHeader *>(&WriteBuffer[0]);
  HeaderWrite->Size = Size;
  HeaderWrite->SizeHeader = SizeHeader;
  HeaderWrite->Width = Width;
  HeaderWrite->Height = Height;
  HeaderWrite->FieldOfViewX = FOVX;
  HeaderWrite->FieldOfViewY = FOVY;

  // Setting the pointers to the data
  Color = &WriteBuffer[OffsetColor];
  Depth = &WriteBuffer[OffsetDepth];
  Object = &WriteBuffer[OffsetObject];
  Map = &WriteBuffer[OffsetMap];
  Read = &ReadBuffer[0];

  IsDataReadable = false;
}

void PacketBuffer::StartWriting(const TMap<FString, uint32> &ObjectToColor, const TArray<FColor> &ObjectColors)
{
  uint32_t Count = 0;
  uint32_t MapSize = 0;
  uint8_t *It = Map;

  // Writing the obejct color map entries to the end of the packet
  for(auto &Elem : ObjectToColor)
  {
    const uint32_t NameSize = Elem.Key.Len();
    const uint32_t ElemSize = sizeof(uint32_t) + 3 * sizeof(uint8_t) + NameSize;
    const FColor &ObjectColor = ObjectColors[Elem.Value];

    // Resize the internal buffer if necessary
    if(Size + MapSize + ElemSize > WriteBuffer.size())
    {
      WriteBuffer.resize(WriteBuffer.size() + 1024 * 1024);
      // Update pointers
      Color = &WriteBuffer[OffsetColor];
      Depth = &WriteBuffer[OffsetDepth];
      Object = &WriteBuffer[OffsetObject];
      Map = &WriteBuffer[OffsetMap];
      HeaderWrite = reinterpret_cast<PacketHeader *>(&WriteBuffer[0]);
    }

    MapEntry *Entry = reinterpret_cast<MapEntry*>(It);
    Entry->Size = ElemSize;

    Entry->R = ObjectColor.R;
    Entry->G = ObjectColor.G;
    Entry->B = ObjectColor.B;

    // Convert name to ANSI and copy it to the packet (no trailing '\0', length is indirectly given by the entry size)
    const char *Name = TCHAR_TO_ANSI(*Elem.Key);
    memcpy(&Entry->FirstChar, Name, NameSize);

    It += ElemSize;
    MapSize += ElemSize;
    ++Count;
  }
  HeaderWrite->MapEntries = Count;
  HeaderWrite->Size = Size + MapSize;
}

void PacketBuffer::DoneWriting()
{
  // Swapping buffers
  LockBuffer.lock();
  IsDataReadable = true;
  WriteBuffer.swap(ReadBuffer);
  Color = &WriteBuffer[OffsetColor];
  Depth = &WriteBuffer[OffsetDepth];
  Object = &WriteBuffer[OffsetObject];
  Map = &WriteBuffer[OffsetMap];
  Read = &ReadBuffer[0];
  HeaderRead = reinterpret_cast<PacketHeader *>(&ReadBuffer[0]);
  HeaderWrite = reinterpret_cast<PacketHeader *>(&WriteBuffer[0]);
  LockBuffer.unlock();
  CVWait.notify_one();
}

void PacketBuffer::StartReading()
{
  // Waits until writing is done
  std::unique_lock<std::mutex> WaitLock(LockRead);
  CVWait.wait(WaitLock, [this] {return IsDataReadable; });

  LockBuffer.lock();
}

void PacketBuffer::DoneReading()
{
  IsDataReadable = false;
  LockBuffer.unlock();
}

void PacketBuffer::Release()
{
  IsDataReadable = true;
  CVWait.notify_one();
}
