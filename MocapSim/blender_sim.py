import bpy
import bmesh
import math
import mathutils
import zmq
import json
import base64
import os

# ==========================================
# 1. CÁC HÀM KHỞI TẠO THÀNH PHẦN
# ==========================================

def create_camera_advanced(name, loc, rot_euler, f_mm, k_px_mm, l_px_mm, cx_px, cy_px, res_w=4608, res_h=2592):
    """
    Tạo Camera dựa trên tham số vật lý và nội tại (Intrinsic)
    - f_mm: Tiêu cự vật lý (mm)
    - k, l: Hệ số tỷ lệ pixel/mm (fx = f*k, fy = f*l)
    """
    cam_data = bpy.data.cameras.new(name)
    cam_obj = bpy.data.objects.new(name, cam_data)
    bpy.context.collection.objects.link(cam_obj)
    
    # Thiết lập Extrinsic
    cam_obj.location = loc
    cam_obj.rotation_euler = rot_euler
    
    # Thiết lập Độ phân giải Scene (Global)
    scene = bpy.context.scene
    scene.render.resolution_x = res_w
    scene.render.resolution_y = res_h
    scene.render.resolution_percentage = 100
    
    # Tính toán Intrinsic (fx, fy)
    fx = f_mm * k_px_mm
    fy = f_mm * l_px_mm
    
    # Ánh xạ sang Blender Sensor
    sensor_width_mm = 36.0 # Cố định cảm biến Full-frame để quy đổi
    cam_data.sensor_fit = 'HORIZONTAL'
    cam_data.sensor_width = sensor_width_mm
    
    # Tính tiêu cự Blender (mm) từ fx
    cam_data.lens = fx * (sensor_width_mm / res_w)
    
    # Xử lý Pixel Aspect Ratio nếu fx != fy
    scene.render.pixel_aspect_x = 1.0
    scene.render.pixel_aspect_y = fy / fx if fx != 0 else 1.0
    
    # Xử lý Principal Point (cx, cy) sang Shift của Blender
    cam_data.shift_x = (cx_px - (res_w / 2.0)) / res_w
    cam_data.shift_y = -((cy_px - (res_h / 2.0)) / res_w) # Trục Y lật ngược, chia cho W vì shift tính theo trục dài nhất
    
    return cam_obj

def create_marker(name, loc, radius, emission_strength):
    """Tạo Marker hình cầu tự phát sáng"""
    bpy.ops.mesh.primitive_uv_sphere_add(radius=radius, location=loc)
    marker = bpy.context.active_object
    marker.name = name
    bpy.ops.object.shade_smooth()
    
    # Vật liệu Emission
    mat = bpy.data.materials.new(name=f"Mat_{name}")
    mat.use_nodes = True
    nodes = mat.node_tree.nodes
    nodes.remove(nodes.get("Principled BSDF"))
    
    emission = nodes.new(type='ShaderNodeEmission')
    emission.inputs['Strength'].default_value = emission_strength
    emission.inputs['Color'].default_value = (1.0, 1.0, 1.0, 1.0)
    
    out = nodes.get("Material Output")
    mat.node_tree.links.new(emission.outputs[0], out.inputs[0])
    marker.data.materials.append(mat)
    
    return marker

def create_checkerboard(name, rows, cols, square_size_mm, loc, align_to_quadrant=False):
    """
    Tạo Checkerboard chính xác tới từng mm.
    Nếu align_to_quadrant=True, góc ván cờ sẽ neo tại (0,0,0) và trải dài vào góc phần tư -xOy.
    """
    size_m = square_size_mm / 1000.0
    width_m = cols * size_m
    height_m = rows * size_m
    
    # Tạo mặt phẳng kích thước 1x1m
    bpy.ops.mesh.primitive_plane_add(size=1.0, location=(0,0,0))
    board = bpy.context.active_object
    board.name = name
    
    # Scale mặt phẳng theo kích thước thật
    board.scale = (width_m, height_m, 1.0)
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
    
    # Đặt vị trí
    if align_to_quadrant:
        # Góc phần tư -xOy: X chạy từ 0 đến -width, Y chạy từ 0 đến +height
        board.location = (-width_m / 2.0, height_m / 2.0, loc[2])
    else:
        board.location = loc
        
    # Tạo vật liệu Checker bằng Procedural Nodes tránh vỡ pixel
    mat = bpy.data.materials.new(name=f"Mat_{name}")
    mat.use_nodes = True
    nodes = mat.node_tree.nodes
    links = mat.node_tree.links
    
    bsdf = nodes.get("Principled BSDF")
    bsdf = nodes.get("Principled BSDF")
    if bsdf:
        nodes.remove(bsdf)
        
    emission = nodes.new(type='ShaderNodeEmission')
    emission.inputs['Strength'].default_value = 1.0
    
    tex_coord = nodes.new('ShaderNodeTexCoord')
    mapping = nodes.new('ShaderNodeMapping')
    mapping.inputs['Scale'].default_value = (cols, rows, 1.0)
    
    checker = nodes.new('ShaderNodeTexChecker')
    checker.inputs['Scale'].default_value = 1.0 
    
    # Ép cứng mã màu Đen - Trắng
    checker.inputs['Color1'].default_value = (1.0, 1.0, 1.0, 1.0)
    checker.inputs['Color2'].default_value = (0.0, 0.0, 0.0, 1.0)
    
    # Nối dây
    links.new(tex_coord.outputs['UV'], mapping.inputs['Vector'])
    links.new(mapping.outputs['Vector'], checker.inputs['Vector'])
    links.new(checker.outputs['Color'], emission.inputs['Color'])
    
    out = nodes.get("Material Output")
    links.new(emission.outputs[0], out.inputs[0])
    
    board.data.materials.append(mat)
    return board

# ==========================================
# 2. TOÁN HỌC TIỆN ÍCH
# ==========================================

def get_look_at_euler(cam_loc, target_loc):
    """Tính toán góc Euler (XYZ) để Camera (hệ Z-Up, nhìn -Z, đỉnh +Y) chĩa thẳng vào Target"""
    direction = mathutils.Vector(target_loc) - mathutils.Vector(cam_loc)
    # Hướng trục -Z của camera về phía vector direction, trục Y hướng lên trên
    rot_quat = direction.to_track_quat('-Z', 'Y')
    return rot_quat.to_euler('XYZ')

def clean_scene():
    """Dọn dẹp môi trường sạch sẽ trước khi setup"""
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete()
    # Tắt đèn môi trường tránh nhiễu
    bpy.context.scene.world.node_tree.nodes["Background"].inputs[1].default_value = 0.0

# ==========================================
# 3. HÀM THỰC THI (MAIN SETUP & NETWORK)
# ==========================================

def run_mocap_simulation():
    clean_scene()
    scene = bpy.context.scene
    
    # 1. Tắt chế độ nền trong suốt (Film Transparent) - BẮT BUỘC
    scene.render.film_transparent = False
    
    # 2. Setup World Node
    if scene.world is None:
        scene.world = bpy.data.worlds.new("Background_World")
        
    world = scene.world
    world.use_nodes = True
    nodes = world.node_tree.nodes
    links = world.node_tree.links
    
    # Xóa sạch node cũ nếu muốn setup lại từ đầu cho sạch
    nodes.clear()
    
    # Tạo node Background
    bg_node = nodes.new(type='ShaderNodeBackground')
    bg_node.inputs['Color'].default_value = (0.2, 0.2, 0.2, 1.0) # Màu xám
    bg_node.inputs['Strength'].default_value = 1.0
    
    # Tạo node World Output
    out_node = nodes.new(type='ShaderNodeOutputWorld')
    
    # NỐI DÂY (Bước này cực kỳ quan trọng)
    links.new(bg_node.outputs['Background'], out_node.inputs['Surface'])
    
    # ---------------- Setup Hình học ----------------
    # 1. Main Checkerboard tại O(0,0,0) nằm trong mặt phẳng -xOy
    main_board = create_checkerboard(
        name="Main_Checker", rows=8, cols=11, square_size_mm=100.0, 
        loc=(0, 0.55, 0.4), align_to_quadrant=False 
    )
    
    # Dựng đứng ván cờ và xoay mặt ra đón Camera
    main_board.rotation_euler = (math.radians(90), 0, math.radians(90))
    # 2. Marker đặt trên một tọa độ xác định của checker (VD: Cột 3, Hàng 2)
    # X âm, Y dương trong vùng -xOy
    marker_loc = (-3 * 0.015, 2 * 0.015, 0.01) 
    main_marker = create_marker(name="Target_Marker", loc=marker_loc, radius=0.005, emission_strength=10.0)
    
    # 3. Hai Checkerboard dùng để calib linh hoạt (Để trôi nổi)
    # calib_board_1 = create_checkerboard("Calib_1", 8, 11, 15.0, loc=(-2, 2, 1.0))
    # calib_board_2 = create_checkerboard("Calib_2", 8, 11, 15.0, loc=(-3, 1, 0.5))
    # calib_board_1.rotation_euler = (math.radians(45), 0, math.radians(20))
    # calib_board_2.rotation_euler = (math.radians(90), 0, math.radians(-30))

    # ---------------- Setup Camera ----------------
    # Giả định tham số intrinsic từ datasheet/calib
    f_mm = 6.0 
    k = 180.0  # pixel/mm
    l = 180.0 
    cx = 4608 / 2
    cy = 2592 / 2

    # Vị trí Camera do user chỉ định
    cam1_loc = (-4.0, 0.0, 1.2)
    cam2_loc = (-4.0, 4.62, 1.2)
    
    # Tự động nội suy góc xoay hướng về Marker
    cam1_rot = get_look_at_euler(cam1_loc, marker_loc)
    cam2_rot = get_look_at_euler(cam2_loc, marker_loc)

    cam1 = create_camera_advanced("Cam_1", cam1_loc, cam1_rot, f_mm, k, l, cx, cy)
    cam2 = create_camera_advanced("Cam_2", cam2_loc, cam2_rot, f_mm, k, l, cx, cy)
    
    cams = [cam1, cam2]
    
    # ---------------- Setup Engine Render ----------------
    scene = bpy.context.scene
    scene.render.engine = 'BLENDER_EEVEE'
    scene.render.image_settings.file_format = 'JPEG'
    scene.render.image_settings.quality = 90
    scene.view_settings.view_transform = 'Standard'

    """
    world_nodes = bpy.context.scene.world.node_tree.nodes
    bg_node = world_nodes.get("Background")

    # inputs[0] là Màu sắc (R, G, B, Alpha). Đặt bằng 1.0 hết là Trắng tinh.
    bg_node.inputs[0].default_value = (1.0, 1.0, 1.0, 1.0) 

    # inputs[1] là Cường độ sáng (Strength). 
    # 1.0 là sáng chuẩn, bạn có thể tăng lên 2.0 hoặc 3.0 nếu ảnh xuất ra vẫn hơi tối.
    bg_node.inputs[1].default_value = 1.0
    """
    
    for cam in cams:
        scene.camera = cam
        temp_path = os.path.abspath(f"mocap_{cam.name}.jpg")
        scene.render.filepath = temp_path
            
        # Thực thi render
        bpy.ops.render.render(write_still=True)
    
    """
    # ---------------- ZMQ IPC Networking ----------------
    port = "5555"
    context = zmq.Context()
    socket = context.socket(zmq.PUB) # Dùng Publish để backend subscribe
    socket.bind(f"tcp://*:{port}")
    print(f"Mocap Simulator: ZMQ Publisher bound to port {port}")

    # Thông số chụp ảnh
    fps = 30
    total_frames = 10 
    delay_between_frames = 1.0 / fps

    print("Bắt đầu chu trình chụp và gửi dữ liệu đồng bộ...")
    
    for frame in range(total_frames):
        # Update logic mô phỏng ở đây (VD: di chuyển Marker)
        # main_marker.location.z = math.sin(frame * 0.5) * 0.5
        
        # Cập nhật context hệ thống cho frame hiện tại
        bpy.context.view_layer.update()
        
        frame_payload = {"frame_id": frame, "images": {}}
        
        # Render đồng bộ từng camera
        for cam in cams:
            scene.camera = cam
            temp_path = f"/tmp/mocap_{cam.name}.jpg"
            scene.render.filepath = temp_path
            
            # Thực thi render
            bpy.ops.render.render(write_still=True)
            
            # Đọc file ảnh dưới dạng binary và encode Base64
            with open(temp_path, "rb") as img_file:
                b64_string = base64.b64encode(img_file.read()).decode('utf-8')
                frame_payload["images"][cam.name] = b64_string
                
            os.remove(temp_path) # Dọn rác
            
        # Đóng gói JSON và ném qua Port
        socket.send_string(json.dumps(frame_payload))
        print(f" Đã gửi thành công Frame {frame} cho {len(cams)} cameras.")
        
        # Giữ đúng nhịp FPS nếu chạy realtime, hoặc bỏ qua để render max tốc độ
        # time.sleep(delay_between_frames) 

    """

# Gọi hàm thực thi
# Chạy dòng này sẽ khóa UI Blender cho đến khi vòng lặp hoàn tất
run_mocap_simulation()