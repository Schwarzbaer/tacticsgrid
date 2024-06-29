sandbox = [
    dict(
        type='square',
        origin=(-15, -15, 0),
        vector_a=(30, 0, 0),
        vector_b=(0, 30, 0),
    ),
    dict(
        type='fence',
        origin=(-15, -15, 0),
        vector_a=(30, 0, 0),
        vector_b=(0, 30, 0),
        vector_c=(0, 0, 3),
    ),
]
playground_towers = [
    dict(
        type='block',
        origin=(-1, -1, 0),
        vector_a=(2, 0, 0),
        vector_b=(0, 2, 0),
        vector_c=(0, 0, 3),
    ),
    dict(
        type='block',
        origin=(-8, -1, 0),
        vector_a=(2, 0, 0),
        vector_b=(0, 2, 0),
        vector_c=(0, 0, 3),
    ),
    dict(
        type='square',
        origin=(-6, -1, 3),
        vector_a=(5, 0, 0),
        vector_b=(0, 2, 0),
    ),
    dict(
        type='square',
        origin=(-1, -1, 3),
        vector_a=(0, -8, -3),
        vector_b=(2, 0, 0),
    ),
]
inner_fence = [
    dict(
        type='fence',
        origin=(-12, 0, 0),
        vector_a=(12, -12, 0),
        vector_b=(12, 12, 0),
        vector_c=(0, 0, 1),
    ),
]
playground = sandbox + playground_towers# + inner_fence


minilevel = [
    dict(
        type='square',
        origin=(-2, -2, 0),
        vector_a=(4, 0, 0),
        vector_b=(0, 4, 0),
    ),
    dict(
        type='square',
        origin=(-2, 2, 0.5),
        vector_a=(2, 0, 0),
        vector_b=(0, -2, 0),
    ),
]
