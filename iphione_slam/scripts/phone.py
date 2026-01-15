#!/usr/bin/env python3

import zmq
import numpy as np
from typing import Tuple
from datetime import datetime
import phone_msg_pb2


class PhonePublisher:
    def __init__(self, ip, port, hwm: int = 1, conflate: bool = True) -> None:
        """Publisher initialization.

        Args:
            ip (str): The IP address of the publisher.
            port (int): The port number of the publisher.
            hwm (int): High water mark for the publisher. Default is 1.
            conflate (bool): Whether to conflate messages. Default is True.
        """

        print("{:-^80}".format(" Phone Publisher Initialization "))
        print(f"Address: tcp://{ip}:{port}")

        # Create a ZMQ context
        self.context = zmq.Context()
        # Create a ZMQ publisher
        self.publisher = self.context.socket(zmq.PUB)
        # Set high water mark
        self.publisher.set_hwm(hwm)
        # Set conflate
        self.publisher.setsockopt(zmq.CONFLATE, conflate)
        # Bind the address
        self.publisher.bind(f"tcp://{ip}:{port}")

        # Init the message
        self.phone = phone_msg_pb2.Phone()

        print("Package Phone")
        print("Message Phone")
        print(
            "{\n\tbytes img = 1;\n\trepeated float pose = 2;\n\trepeated float force = 3;\n\trepeated float node = 4;\n}"
        )

        print("Phone Publisher Initialization Done.")
        print("{:-^80}".format(""))

    def publishMessage(
        self,
        color_img_bytes: bytes = b"",
        depth_img: np.ndarray = np.array([]),
        depth_width: int = 0,
        depth_height: int = 0,
        pose: np.ndarray = np.array([]),
    ) -> None:
        """Publish the message.

        Args:
            color_img_bytes: The image captured by the camera.
            depth_img: The depth image captured by the camera.
            pose: The pose of the marker (numpy array or list).
        """

        # Set the message
        self.phone.timestamp = datetime.now().timestamp()
        self.phone.color_img = color_img_bytes
        self.phone.depth_img = depth_img.flatten().tolist()
        self.phone.depth_width = depth_width
        self.phone.depth_height = depth_height
        self.phone.pose[:] = pose.flatten().tolist()

        # Publish the message
        self.publisher.send(self.phone.SerializeToString())

    def close(self):
        """Close ZMQ socket and context to prevent memory leaks."""
        if hasattr(self, "publisher") and self.publisher:
            self.publisher.close()
        if hasattr(self, "context") and self.context:
            self.context.term()


class PhoneSubscriber:
    def __init__(self, ip, port, hwm: int = 1, conflate: bool = True) -> None:
        """Subscriber initialization.

        Args:
            ip (str): The IP address of the subscriber.
            port (int): The port number of the subscriber.
            hwm (int): High water mark for the subscriber. Default is 1.
            conflate (bool): Whether to conflate messages. Default is True.
        """

        print("{:-^80}".format(" Phone Subscriber Initialization "))
        print(f"Address: tcp://{ip}:{port}")

        # Create a ZMQ context
        self.context = zmq.Context()
        # Create a ZMQ subscriber
        self.subscriber = self.context.socket(zmq.SUB)
        # Set high water mark
        self.subscriber.set_hwm(hwm)
        # Set conflate
        self.subscriber.setsockopt(zmq.CONFLATE, conflate)
        # Connect the address
        self.subscriber.connect(f"tcp://{ip}:{port}")
        # Subscribe the topic
        self.subscriber.setsockopt_string(zmq.SUBSCRIBE, "")
        # Set poller
        self.poller = zmq.Poller()
        self.poller.register(self.subscriber, zmq.POLLIN)

        # Init the message
        self.phone = phone_msg_pb2.Phone()

        print("Package Phone")
        print("Message Phone")
        print(
            "{\n\tbytes img = 1;\n\trepeated float pose = 2;\n\trepeated float force = 3;\n\trepeated float node = 4;\n}"
        )

        print("Phone Subscriber Initialization Done.")
        print("{:-^80}".format(""))

    # def subscribeMessage(self, timeout: int = 100) -> Tuple[bytes, np.ndarray, np.ndarray, np.ndarray]:
    #     """Subscribe the message.

    #     Args:
    #         timeout: Maximum time to wait for a message in milliseconds. Default is 100ms.

    #     Returns:
    #         color_img: The image captured by the camera.
    #         depth_img: The depth image captured by the camera.
    #         pose: The pose of the marker (numpy array).

    #     Raises:
    #         zmq.ZMQError: If no message is received within the timeout period.
    #     """

    #     # Receive the message
    #     socks = dict(self.poller.poll(timeout))
    #     if self.subscriber in socks and socks[self.subscriber] == zmq.POLLIN:
    #         self.phone.ParseFromString(self.subscriber.recv())
    #     else:
    #         raise zmq.ZMQError("No message received within the timeout period.")
    #     return (
    #         self.phone.color_img,
    #         np.array(self.phone.depth_img).reshape(
    #             self.phone.depth_height, self.phone.depth_width
    #         ),
    #         np.array(self.phone.pose),
    #     )

    def subscribeMessage(self, timeout: int = 100) -> Tuple[float, bytes, bytes, int, int, list, list, list, list]:
      
        socks = dict(self.poller.poll(timeout))
        if self.subscriber in socks and socks[self.subscriber] == zmq.POLLIN:
            # 接收多部分消息
            try:
                protobuf_data = self.subscriber.recv(zmq.DONTWAIT)
                
                # 解析protobuf数据
                self.phone.ParseFromString(protobuf_data)
                
            except zmq.Again:
                raise zmq.ZMQError("接收多部分消息时出错")
        else:
            raise zmq.ZMQError("No message received within the timeout period.")
        
        # 处理depth_img - 现在是bytes类型
        if self.phone.depth_img and self.phone.depth_width > 0 and self.phone.depth_height > 0:
            # 将bytes转换为numpy数组 - 深度图像是uint16格式
            depth_array = np.frombuffer(self.phone.depth_img, dtype=np.uint16)
            depth_img = depth_array.reshape(self.phone.depth_height, self.phone.depth_width)
        else:
            # 如果没有深度数据，返回空数组
            depth_img = np.array([])
        
        return (
            self.phone.timestamp,
            self.phone.color_img,
            self.phone.depth_img,
            self.phone.depth_width,
            self.phone.depth_height,
            np.array(self.phone.local_pose),
            np.array(self.phone.global_pose),
            np.array(self.phone.velocity),
            np.array(self.phone.rotation_rate)
           
        )

    def close(self):
        """Close ZMQ socket and context to prevent memory leaks."""
        if hasattr(self, "subscriber") and self.subscriber:
            self.subscriber.close()
        if hasattr(self, "context") and self.context:
            self.context.term()
