# Image Binarization Using Fixed Threshold

## Overview
This project implements a **hardware-accelerated image binarization** system using a **System on Chip (SoC)** for optimized processing. The main focus is on the **data flow** from the **SD card to the Processing System (PS), then to the Programmable Logic (PL) via Direct Memory Access (DMA), and back to the SD card** after processing.

## Data Flow & Processing
1. **Image Acquisition:**
   - A **24-bit grayscale BMP image** is stored on an **SD card**.
   - The **Processing System (PS)** reads the image from the SD card and stores it in **DDR memory**.

2. **Data Transfer to Programmable Logic (PL):**
   - The **DMA controller** is used to efficiently transfer image data from **DDR memory to the PL**.
   - **AXI4-Stream FIFO** buffers the data during the transfer process.

3. **Binarization Process in PL:**
   - The **binarization module** processes the image data **pixel-by-pixel**, comparing pixel values to a **fixed threshold** (default: 128).
   - The output pixels are converted to either **0 (black) or 255 (white)**.

4. **Data Transfer Back to PS:**
   - Once binarization is complete, the **processed image data is transferred back** to **DDR memory** via DMA.
   - The **PS retrieves the processed image** and prepares it for storage.

5. **Writing Back to SD Card:**
   - The **PS writes the binarized image** to a **new BMP file** on the SD card.
   - The processed image can then be accessed and analyzed externally.

## System Components
### **1. Processing System (PS)**
- Reads and writes image data from/to SD card.
- Manages **DMA transfers** between DDR and PL.

### **2. Programmable Logic (PL)**
- Performs **binarization of the image**.
- Uses an **AXI4-Stream interface** for fast processing.

### **3. Direct Memory Access (DMA)**
- **Transfers image data** efficiently between PS and PL.
- Ensures fast, low-latency communication.

## Input & Output
### **Input:**
- 24-bit BMP grayscale image stored on an **SD card**.
- User-defined **fixed threshold** (default: 128).

### **Output:**
- 24-bit BMP **black-and-white** image saved on the SD card.
