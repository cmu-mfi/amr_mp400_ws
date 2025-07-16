from PySide6.QtWidgets import (QLabel, QGraphicsView, QGraphicsScene)
from PySide6.QtCore import Qt
from PySide6.QtGui import QPainter, QPixmap

class CameraView(QGraphicsView):
    def __init__(self, title):
        super().__init__()
        self.setScene(QGraphicsScene())
        self.setRenderHint(QPainter.Antialiasing)
        self.setRenderHint(QPainter.SmoothPixmapTransform)
        self.setAlignment(Qt.AlignCenter)
        self.title = title
        self.current_image = None
        
        # Title label
        self.title_label = QLabel(title)
        self.title_label.setStyleSheet("""
            QLabel {
                color: white;
                font-size: 24px;
                background-color: rgba(0, 0, 0, 150);
                padding: 5px;
            }
        """)
        self.title_label.setAlignment(Qt.AlignCenter)
        self.title_label.setFixedWidth(self.width())
        self.title_label.move(10, 10)
        
    def set_image(self, qimage):
        self.current_image = qimage
        
    def update(self):
        if self.current_image:
            pixmap = QPixmap.fromImage(self.current_image)
            self.scene().clear()
            self.scene().addPixmap(pixmap)
            self.setSceneRect(pixmap.rect())
            self.fitInView(self.sceneRect(), Qt.KeepAspectRatio)
            
            # Update title label position
            self.title_label.setFixedWidth(self.width())
            self.title_label.move(10, 10)