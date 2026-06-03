# InferenceResult.msg
# Results from object detection / weather classification model
drone_interfaces/Detection2D[] detections
string task_type          # "object_detection", "weather_classification"
float32 inference_time_ms
string model_name
