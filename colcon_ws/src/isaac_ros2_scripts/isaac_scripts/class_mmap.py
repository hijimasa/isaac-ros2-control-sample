import os
import struct
import mmap

FLOAT_SIZE = 4  #1FLOAT = 4byte
MAP_SIZE = 4 * FLOAT_SIZE * 256 # command position velocity force
MAP_FILE_NAME = "/dev/shm/isaac_ros2_control_data"

class classMMap:
    def __init__(self) -> None:
        self._mm = None
        if os.path.exists(MAP_FILE_NAME):
            self._readMMapFile()
        else:
            self._createMMapFile()

            self._readMMapFile()

        print(" __init__ comp.")


    def _createMMapFile(self):        
        with open(MAP_FILE_NAME, mode="wb") as file:
            initStr = '00' * MAP_SIZE
            initByte = bytes.fromhex(initStr)
            file.write(initByte)

        print("_createMMapFile fin.")

    def _readMMapFile(self):
        with open(MAP_FILE_NAME, mode="r+b") as file:
            self._mm = mmap.mmap(file.fileno(), 0)
            self._mm.seek(0)

        print("_readMMapFile fin.")

    def ReadFloat(self, adr:int):
        try:
            self._mm.seek(adr*FLOAT_SIZE)
            bytes = self._mm.read(FLOAT_SIZE)
            val = struct.unpack('<f', bytes)[0]
            self._mm.seek(0)
            return val
        except Exception as e:
            print("ReadShort except" + str(e).replace('\n',''))
            return

    def WriteFloat(self, adr:int, data:float) -> None:
        try:
            bytes = struct.pack('<f', data)

            for i in range(FLOAT_SIZE):
                self._mm[adr*FLOAT_SIZE + i] = bytes[i]
        except Exception as e:
            print("WriteShort except" + str(e).replace('\n',''))

    def __del__(self):
        self._mm.close()