# Créditos de los assets

Todo está copiado dentro de `models/` para que los mundos abran sin internet.

## Texturas — ambientCG (CC0)

De <https://ambientcg.com>, versión 1K-JPG (color, normal GL y rugosidad).
Licencia CC0: dominio público, no exige atribución.

| Carpeta en `models/x500_texturas/` | Material de ambientCG |
|---|---|
| `adoquin` | PavingStones070 |
| `pasto` | Grass004 |
| `concreto_oscuro` | Concrete031 |
| `concreto_claro` | Concrete034 |
| `banqueta` | Concrete047A |
| `calle` | Road007 |
| `asfalto` | Asphalt031 |
| `tierra_pasto` | Ground037 |
| `lecho_rio` | Ground103 |
| `roca` | Rock030 |
| `madera` | Planks037A |
| `ladrillo` | Bricks106A |
| `grava` | Gravel022 |

## Modelos — Gazebo Fuel (OpenRobotics)

De <https://app.gazebosim.org/OpenRobotics>.

| Carpeta en `models/` | Modelo de Fuel | Licencia |
|---|---|---|
| `house_1`, `house_2`, `house_3` | House 1, House 2, House 3 | CC0 |
| `apartment` | Apartment | CC0 |
| `water_tower` | Water tower | CC0 |
| `lamp_post` | Lamp Post | CC0 |
| `oak_tree`, `pine_tree` | Oak tree, Pine Tree | CC0 |
| `fountain` | Fountain | CC0 |
| `bench_long` | FoodCourtBenchLong | CC0 |
| `hatchback_blue`, `hatchback_red` | Hatchback blue, Hatchback red | CC0 |
| `suv`, `pickup` | SUV, Pickup | CC0 |
| `truck_delivery` | TruckDelivery | CC BY 4.0, © Open Robotics |
| `falling_rock_1` | Falling Rock 1 | CC BY 4.0, © Open Robotics |

Cambios hechos a los modelos de Fuel: rutas `model://` a la carpeta local,
materiales de Ogre 1 (que Gazebo Harmonic ignora y dejaban las casas negras)
cambiados por PBR con las mismas texturas, y texturas que apuntaban a
internet o a otra carpeta copiadas junto a la malla.

## Hecho para este paquete

`x500_aruco`: marcadores DICT_5X5_250 generados con OpenCV por
`tools/make_aruco.py`.


`x500_mallas` (piso, puente, farmacia, cercas...) lo genera
`tools/build_worlds.py`, igual que las balizas, el escenario y las marcas del
pilar que van dentro de cada `.sdf`.
