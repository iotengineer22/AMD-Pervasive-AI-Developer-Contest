import onnxruntime

# Add other imports
# ...

# Load inputs and do preprocessing
# ...

# Create an inference session using the Vitis-AI execution provider

session = onnxruntime.InferenceSession(
'yolox_nano_onnx_pt.onnx',
providers=["VitisAIExecutionProvider"],
provider_options=[{"config_file":"/usr/bin/vaip_config.json"}])

input_shape = session.get_inputs()[0].shape
input_name = session.get_inputs()[0].name
print(input_shape)
print(input_name)

# Load inputs and do preprocessing by input_shape
# input_data = [...]
# result = session.run([], {input_name: input_data})