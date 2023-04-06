# def reverse_coords(coords, max_height):
#     for pos in coords:
#         pos[1] = max_height - pos[1]
# def shift_origin(coords, origin):
#     for pos in coords:
#         pos[0] = pos[0] - origin[0]
#         pos[1] = pos[1] - origin[1]
# def center_coords(coords):
#     for pos in coords:
#         pos[0] += 0.5
#         pos[1] += 0.5
# def scale_coords(coords, scale=[1,1]):
#     for pos in coords:
#         pos[0] *= scale[0]
#         pos[1] *= scale[1]

# def array_to_real_position(coords, max_height):
#     for pos in coords:
#         #reverse array row indices
#         pos[1] = max_height - pos[1] 

#         #shift origin
#         pos[0] = pos[0] - 4
#         pos[1] = pos[1] - 4

#         #scale
#         pos[0] *= 3.5/8
#         pos[1] *= 3.5/8

# positions = surround_fire(field, 4)
# print(positions)
# for p in positions:
#     p[1] = 10 - p[1]

#     p[0] = p[0] - 5
#     p[1] = p[1] - 5

#     p[0] *= 3.5/10
#     p[1] *= 3.5/10