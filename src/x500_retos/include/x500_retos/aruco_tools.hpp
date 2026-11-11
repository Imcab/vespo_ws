// =============================================================================
// aruco_tools.hpp: de un ArUco visto por la cámara de abajo a metros en NED.
//
// La cámara del x500_cam_abajo mira derecho hacia abajo (si el dron va
// nivelado). En su imagen:
//
//          arriba de la imagen = el FRENTE del dron
//      ┌──────────────────────────┐
//      │            ▲ adelante    │       x_norm = (u - cx) / fx
//      │            │             │       y_norm = (v - cy) / fy
//      │     ───────┼──────► derecha        (tangentes del ángulo desde el centro)
//      │            │             │
//      └──────────────────────────┘
//
// Cada píxel es un RAYO que sale de la cámara. En el cuerpo del dron (FRD:
// x adelante, y derecha, z abajo) el rayo del marcador es
//     (-y_norm, x_norm, 1)        (v crece hacia abajo de la imagen = hacia atrás)
//
// OJO, la cámara NO tiene gimbal: va pegada al dron. Cuando el dron se inclina
// 15° para acelerar, a 12 m de altura la imagen se corre ~3 m. Por eso el rayo
// se gira con la actitud COMPLETA (el cuaternión q de PX4: roll, pitch y yaw)
// y luego se alarga hasta que baja `height` metros: ahí está el marcador.
// =============================================================================
#pragma once

#include <array>
#include <cmath>

#include "x500_retos/msg/aruco.hpp"

namespace x500
{

// Gira un vector del cuerpo (FRD) a NED con el cuaternión q = {w, x, y, z}.
inline std::array<double, 3> body_to_ned(const std::array<float, 4> & q, const std::array<double, 3> & v)
{
  const double w = q[0], x = q[1], y = q[2], z = q[3];
  return {
    (1 - 2 * (y * y + z * z)) * v[0] + 2 * (x * y - w * z) * v[1] + 2 * (x * z + w * y) * v[2],
    2 * (x * y + w * z) * v[0] + (1 - 2 * (x * x + z * z)) * v[1] + 2 * (y * z - w * x) * v[2],
    2 * (x * z - w * y) * v[0] + 2 * (y * z + w * x) * v[1] + (1 - 2 * (x * x + y * y)) * v[2],
  };
}

// Desplazamiento {norte, este} en metros del marcador respecto al dron.
//   height: cuánto está la cámara arriba del marcador (m)
//   q:      actitud del dron (px4_.attitude().q)
inline std::array<double, 2> marker_offset_ned(
  const x500_retos::msg::Aruco & m, double height, const std::array<float, 4> & q)
{
  const auto ray = body_to_ned(q, {-m.y_norm, m.x_norm, 1.0});
  if (ray[2] < 0.1) {
    return {0.0, 0.0};   // el rayo casi no baja (dron muy inclinado): no confiar
  }
  const double t = height / ray[2];
  return {ray[0] * t, ray[1] * t};
}

}  // namespace x500
