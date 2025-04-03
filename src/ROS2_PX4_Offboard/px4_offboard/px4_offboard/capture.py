import cv2
import numpy as np
import os

def detect_objects(image):
    base_directory = os.path.dirname(__file__)

    cfg_file = os.path.join(base_directory, "yolov4-tiny.cfg")
    weights_file = os.path.join(base_directory, "yolov4-tiny_last.weights")
    class_file = os.path.join(base_directory, "classes.txt")

    with open(class_file, "r") as f:
        classes = f.read().strip().split("\n")

    net = cv2.dnn.readNetFromDarknet(cfg_file, weights_file)

    height, width = image.shape[:2]
    blob = cv2.dnn.blobFromImage(image, 1/255.0, (416, 416), swapRB=True, crop=False)
    net.setInput(blob)

    layer_names = net.getUnconnectedOutLayersNames()
    outputs = net.forward(layer_names)

    boxes, confidences, class_ids = [], [], []

    for output in outputs:
        for detection in output:
            scores = detection[5:]
            class_id = np.argmax(scores)
            confidence = scores[class_id]

            if confidence > 0.5:
                center_x, center_y, w, h = (detection[:4] * [width, height, width, height]).astype("int")
                x = int(center_x - w / 2)
                y = int(center_y - h / 2)

                boxes.append([x, y, w, h])
                confidences.append(float(confidence))
                class_ids.append(class_id)

    indices = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)

    if len(indices) > 0:
        for i in indices.flatten():
            x, y, w, h = boxes[i]
            label = f"{classes[class_ids[i]]}: {confidences[i]:.2f}"
            color = (0, 255, 0)
            cv2.rectangle(image, (x, y), (x + w, y + h), color, 2)
            cv2.putText(image, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

    return image


'''
import cv2
import numpy as np
import os

def detect_objects(image):
    base_directory = os.path.dirname(__file__)

    cfg_file = os.path.join(base_directory, "yolov4-tiny.cfg")
    weights_file = os.path.join(base_directory, "yolov4-tiny_last.weights")
    class_file = os.path.join(base_directory, "classes.txt")

    with open(class_file, "r") as f:
        classes = f.read().strip().split("\n")

    net = cv2.dnn.readNetFromDarknet(cfg_file, weights_file)

    height, width = image.shape[:2]
    blob = cv2.dnn.blobFromImage(image, 1/255.0, (416, 416), swapRB=True, crop=False)
    net.setInput(blob)

    layer_names = net.getUnconnectedOutLayersNames()
    outputs = net.forward(layer_names)

    boxes, confidences, class_ids = [], [], []

    for output in outputs:
        for detection in output:
            scores = detection[5:]
            class_id = np.argmax(scores)
            confidence = scores[class_id]

            if confidence > 0.5:
                center_x, center_y, w, h = (detection[:4] * [width, height, width, height]).astype("int")
                x = int(center_x - w / 2)
                y = int(center_y - h / 2)

                boxes.append([x, y, w, h])
                confidences.append(float(confidence))
                class_ids.append(class_id)

    indices = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)

    if len(indices) > 0:
        for i in indices.flatten():
            x, y, w, h = boxes[i]
            label = f"{classes[class_ids[i]]}: {confidences[i]:.2f}"
            color = (0, 255, 0)
            cv2.rectangle(image, (x, y), (x + w, y + h), color, 2)
            cv2.putText(image, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

    # Criar janela menor
    cv2.namedWindow("Object Detection", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Object Detection", 800, 600)  # Ajuste conforme necessário

    while True:
        cv2.imshow("Object Detection", image)
        if cv2.waitKey(1) & 0xFF == ord('q'):  # Pressione 'q' para sair
            break

    cv2.destroyAllWindows()
'''