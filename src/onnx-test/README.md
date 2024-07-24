## Model File Download(yolox_nano_onnx_pt.onnx)

The Model file can be downloaded from Vitis AI:

This time, we used the models provided by Vitis AI. Pre-trained and quantized models are provided as samples by Xilinx (AMD), converted from PyTorch models to ONNX:

https://github.com/Xilinx/Vitis-AI/tree/master/model_zoo/model-list/pt_yolox-nano_3.5


Download and unzip the YOLOX sample model:

The "yolox_nano_onnx_pt.onnx" file is in the "quantized" folder. Transfer this file to the KR260 without further compilation.

```example
wget https://www.xilinx.com/bin/public/openDownload?filename=pt_yolox-nano_3.5.zip
unzip openDownload\?filename\=pt_yolox-nano_3.5.zip
```


## Model File Download(yolox_nano.onnx)

The Model file can be downloaded from YOLOX:
This time, we used the models provided by YOLOX. 
https://yolox.readthedocs.io/en/latest/demo/onnx_readme.html
