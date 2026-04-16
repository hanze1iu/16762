import cv2
import numpy as np
from geometry_msgs.msg import PoseStamped

COLOR_PALETTE = [
    (10, 10, 255),
    (255, 56, 56),
    (157, 255, 151),
    (255, 112, 31),
    (100, 178, 29),
    (207, 210, 49),
    (72, 249, 10),
    (146, 204, 23),
    (61, 219, 134),
]


def parse_results(results):

    detections = []
    idx = 0
    seen_objects = [] # want to avoid duplicate detections, assume only one object of each type is in the scene
    for result in results:

        boxes = result.boxes
        masks = result.masks
        names = result.names

        if boxes is None or len(boxes) == 0:
            continue

        for box, mask in zip(boxes, masks):

            cls_id = int(box.cls)

            if box.cls in seen_objects: # filter out duplicate detections
                continue
            seen_objects.append(cls_id)

            confidence = float(box.conf)
            bounding_box = box.xyxy[0].cpu().numpy().round().astype(int) # [x_min, y_min, x_max, y_max]
            mask = mask.xy[0].astype(int) # formatted as a set of N polygon points in [x, y] pixel format, dim Nx2)
            centroid = (int(mask[:,0].mean()), int(mask[:,1].mean()))
            label = names[cls_id]

            print(f"Detected: {label}, Confidence: {confidence:.2f}, Box: {bounding_box.tolist()}")

            detections.append({
                "id": idx,
                "class_id": cls_id,
                "label": label,
                "confidence": confidence,
                "bounding_box": bounding_box,
                "mask": mask,
                "centroid": centroid,
                "color": COLOR_PALETTE[cls_id]
            })
            idx += 1

    return detections


def pixel_to_3d(xy_pix, z_depth, camera_info):

    camera_mat = np.reshape(camera_info.k, (3, 3))

    z_depth = z_depth / 1000 # convert to meters
    # print(camera_info)

    # print(camera_info)
    x_pix, y_pix = xy_pix
    f_x = camera_mat[0,0]
    c_x = camera_mat[0,2]
    f_y = camera_mat[1,1]
    c_y = camera_mat[1,2]
    x_out = ((x_pix - c_x) * z_depth) / f_x
    y_out = ((y_pix - c_y) * z_depth) / f_y
    xyz_out = np.array([x_out, y_out, z_depth])

    print(xyz_out*100, 'cm') # convert to cm

    return xyz_out

def get_pose_msg(timestamp, frame_id, xyz_out):
    msg = PoseStamped()

    msg.header.stamp = timestamp
    msg.header.frame_id = frame_id

    msg.pose.position.x = xyz_out[0]
    msg.pose.position.y = xyz_out[1]
    msg.pose.position.z = xyz_out[2]

    # set orientation to Identity since it doesn't matter too much, only care about position
    msg.pose.orientation.x = 0.0
    msg.pose.orientation.y = 0.0
    msg.pose.orientation.z = 0.0
    msg.pose.orientation.w = 1.0

    return msg


def visualize_detections_masks(part, detections, rgb_image, depth_image=None):
    font_scale = 0.6
    line_color = [0, 0, 0]
    line_width = 1
    alpha=0.4
    font = cv2.FONT_HERSHEY_PLAIN

    annotated_rgb_img = rgb_image.copy()
    mask_overlay = rgb_image.copy()

    if detections is None:
        open_cv2_window(part, annotated_rgb_img, depth_image)
        return


    # create the polygon that defines the detection mask, apply it to the mask overlay img
    for detection in detections:
        mask = detection["mask"]
        cv2.fillPoly(mask_overlay, [mask], detection["color"])

    # combine the mask overlay with the original image to get colored, transparent masks over the original image
    annotated_rgb_img = cv2.addWeighted(annotated_rgb_img, 1 - alpha, mask_overlay, alpha, 0)

    # loop back through to do additional centroid and text annotations
    for detection in detections:

        centroid = detection["centroid"]
        # y, x img coords vs x, y array coords
        centroid_output_string = f"{centroid[0]}, {centroid[1]}"
        text_x = centroid[0] + 10
        text_y = centroid[1]
        cv2.circle(annotated_rgb_img, centroid, 4, (255, 255, 255), -1) # y, x
        cv2.putText(
            annotated_rgb_img,
            centroid_output_string,
            (text_x, text_y),
            font,
            font_scale,
            line_color,
            line_width,
            cv2.LINE_AA,
        )

        box = detection["bounding_box"]
        class_label = detection["label"]
        confidence = detection["confidence"]
        x_min, y_min, x_max, y_max = box
        detection_output_string = "{0}, {1:.2f}".format(class_label, confidence)

        label_background_border = 2
        (label_width, label_height), baseline = cv2.getTextSize(
            detection_output_string, font, font_scale, line_width
        )
        label_x_min = x_min
        label_y_min = y_min
        label_x_max = x_min + (label_width + (2 * label_background_border))
        label_y_max = y_min + (label_height + baseline + (2 * label_background_border))

        text_x = label_x_min + label_background_border
        text_y = (label_y_min + label_height) + label_background_border

        cv2.rectangle(
            annotated_rgb_img,
            (label_x_min, label_y_min),
            (label_x_max, label_y_max),
            (255, 255, 255),
            cv2.FILLED,
        )
        cv2.putText(
            annotated_rgb_img,
            detection_output_string,
            (text_x, text_y),
            font,
            font_scale,
            line_color,
            line_width,
            cv2.LINE_AA,
        )

    open_cv2_window(part, annotated_rgb_img, depth_image)
    # for centroid in centroids:
    #     cv2.circle(annotated_img, centroid, 5, (255, 255, 255), -1)

def open_cv2_window(part, color_img=None, depth_img=None):

    subplots = []
    if color_img is not None:
        color_img = cv2.cvtColor(color_img, cv2.COLOR_RGB2BGR)
        subplots.append(color_img)

    if depth_img is not None:
        depth_img = np.nan_to_num(depth_img)
        if part==1:
            depth_img = np.clip(depth_img, 70, 500) # ideal operating range of D405
        elif part==2:
            depth_img = np.clip(depth_img, 300, 3000) # ideal operating range of D405
        depth_img = cv2.normalize(depth_img, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        depth_img = cv2.applyColorMap(depth_img, cv2.COLORMAP_JET)
        subplots.append(depth_img)

    display_img = np.concatenate(subplots, axis=1)
    # print(color_img.shape, depth_img.shape, display_img.shape)

    cv2.imshow('display_img', display_img)
    cv2.waitKey(3)
