# Un ArUco que vio la cámara de abajo (lo publica detector_aruco).
#
# u, v: pixeles, con el origen arriba a la izquierda (u a la derecha, v hacia
# abajo). x_norm, y_norm: lo mismo pero "normalizado" con la calibración de la
# cámara: la tangente del ángulo desde el centro de la imagen. Multiplicado
# por la altura de la cámara sobre el marcador da metros (ver aruco_tools.hpp).

int32 id
float32 u          # [px] centro del marcador
float32 v          # [px]
float32 x_norm     # (u - cx) / fx: hacia la derecha de la imagen
float32 y_norm     # (v - cy) / fy: hacia abajo de la imagen
float32 size_px    # [px] lado promedio del marcador
