import numpy as np
import pandas as pd
import folium
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp


def load_data_from_csv(filename):
    data = pd.read_csv(filename)
    locations = [(row['Latitude'], row['Longitude']) for _, row in data.iterrows()]
    return locations


def create_data_model():
    data = {}
    data['locations'] = load_data_from_csv('locations.csv')
    data['num_vehicles'] = 1
    data['depot'] = 0
    return data

def main():
    data = create_data_model()


    manager = pywrapcp.RoutingIndexManager(len(data['locations']), data['num_vehicles'], data['depot'])
    routing = pywrapcp.RoutingModel(manager)


    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return np.linalg.norm(np.array(data['locations'][from_node]) - np.array(data['locations'][to_node]))

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)


    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)


    solution = routing.SolveWithParameters(search_parameters)


    if solution:
        print_solution(manager, routing, solution, data)

def print_solution(manager, routing, solution, data):
    print('Расстояние: {} км'.format(solution.ObjectiveValue()))
    index = routing.Start(0)
    plan_output = 'Маршрут:'
    route_distance = 0
    route = []
    while not routing.IsEnd(index):
        plan_output += ' {} ->'.format(manager.IndexToNode(index))
        route.append(manager.IndexToNode(index))
        previous_index = index
        index = solution.Value(routing.NextVar(index))
        route_distance += routing.GetArcCostForVehicle(previous_index, index, 0)
    route.append(manager.IndexToNode(index))
    plan_output += ' {}\n'.format(manager.IndexToNode(index))
    print(plan_output)
    print('Общее расстояние: {} км'.format(route_distance))


    plot_route(data, route)

def plot_route(data, route):
    map_center = np.mean(data['locations'], axis=0)
    m = folium.Map(location=map_center, zoom_start=12)


    folium.Marker(location=data['locations'][0], popup='Склад', icon=folium.Icon(color='green')).add_to(m)


    for i, loc in enumerate(data['locations'][1:]):
        folium.Marker(location=loc, popup=f'Клиент {i+1}', icon=folium.Icon(color='blue')).add_to(m)


    route_points = [data['locations'][i] for i in route]
    folium.PolyLine(locations=route_points, color='purple').add_to(m)

    m.save('route_map.html')

if __name__ == '__main__':
    main()
