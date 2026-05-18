from src.grid_map import GridMap
from src.state import DroneState
from src.energy_epea import epea_star_energy

def test_drone_must_recharge():
    grid = [
        [0, 0, 0],
        [0, 0, 0],
    ]
    chargers = {(1, 0)}

    starts = (DroneState((0,0), 1),)
    goals  = ((2,0),)
    gm = GridMap(grid, chargers)

    path, _ = epea_star_energy(gm, starts, goals)
    assert path is not None
    # dron musi przejść przez charger (1,0) aby dotrzeć do celu
    assert any(state[0].position == (1,0) for state in path)