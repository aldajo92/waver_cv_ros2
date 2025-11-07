# Custom GStreamer Plugin

## Build and Install Inside Docker

```bash
# Enter docker container
./scripts/shell.sh

# Inside container
cd /ros2_ws/../gst_custom_plugin
make
make install
make test

# Verify plugin is loaded
gst-inspect-1.0 myfilter
```

## Use in Launch File

Update `main_gscam.launch.py`:

```python
gscam_config = 'v4l2src device=/dev/video0 ! video/x-raw,width=640,height=480 ! videoconvert ! video/x-raw,format=RGB ! myfilter ! videoconvert ! video/x-raw,format=RGB'
```

## Modify Processing

Edit `gstmyfilter.cpp` in the `gst_my_filter_transform_frame_ip` function:
- Access frame data via `data` pointer
- `width`, `height`, `stride` give frame dimensions
- Process pixels directly (RGB format: 3 bytes per pixel)
- Return `GST_FLOW_OK` when done

## Examples

**Brightness:**
```cpp
data[i * stride + j] = CLAMP(data[i * stride + j] + 20, 0, 255);
```

**Edge Detection:**
```cpp
gint dx = abs(data[idx] - data[idx + 3]);
gint dy = abs(data[idx] - data[idx + stride]);
data[idx] = CLAMP(dx + dy, 0, 255);
```

**Color Filter:**
```cpp
// Keep only red channel
data[i * stride + j + 1] = 0;  // Green = 0
data[i * stride + j + 2] = 0;  // Blue = 0
```

