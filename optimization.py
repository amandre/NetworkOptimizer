import networkx as nx
import xml.etree.ElementTree as ET
import math
import matplotlib.pyplot as plt
import numpy

'''
assumptions:
 - setupCost = 0 (for existing edge) or setupCost = calculate_cost(city1,city2) (for new edge)
 - capacity = 1 (by default each path uses only one edge)
 #EDIT - cap = 10 (def cap=1)
'''

cap = 10 # link (module) capacity [Gbps]
#global budget
budget = 100000

def calculate_cost (graph, city1,city2):
    # calculates cost of a link added between 2 locations
    # calculates distance in kilometers based on Cartesian coordinates
    # x describes East or West, y describes North or South
    x1 = graph.node[city1]['x']
    y1 = graph.node[city1]['y']
    x2 = graph.node[city2]['x']
    y2 = graph.node[city2]['y']

    R = 6371e3 # radius in meters
    fi1 = math.radians(y1)
    fi2 = math.radians(y2)
    dfi = math.radians(y2-y1)
    ddelta = math.radians(x2-x1)

    a = math.sin(dfi/2) * math.sin(dfi/2) + math.cos(fi1) * math.cos(fi2) * math.sin(ddelta/2) * math.sin(ddelta/2);
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    distance = int(R * c/1000) # distance in kilometers
    return distance^2 # cost of a newly installed link between 2 locations

def draw_graph(xml_file, cap):
    # builds a graph with existing vertices and edges
    topo = ET.parse(xml_file)
    root = topo.getroot()
    graph = nx.DiGraph() #for directed simple graph
    i = 0

    for v in root.iter('{http://sndlib.zib.de/network}node'):
        i += 1
        graph.add_node(v.attrib['id'], no=i, x=float(v[0][0].text), y=float(v[0][1].text))

    n = len(graph.nodes)
    demand = numpy.zeros((n,n))

    for vs in graph.nodes:
        vsrc_id = graph.node[vs]["no"]
        for vd in graph.nodes:
            vdst_id = graph.node[vd]["no"]
            if vsrc_id != vdst_id:
                demand[vsrc_id-1][vdst_id-1] = abs(vdst_id-vsrc_id)*10 # [Gbps]

    for e in root.iter('{http://sndlib.zib.de/network}link'):
        #creating a pair of edges (with attributes such as an id, sd, td, capacity and setupCost)
        num0 = graph.node[e[0].text]['no']-1
        num1 = graph.node[e[1].text]['no']-1
        graph.add_edge(e[0].text,e[1].text,id=e.attrib['id'],d=demand[num0][num1],capacity=cap,setupCost=0)
        graph.add_edge(e[1].text,e[0].text,id=e.attrib['id'],d=demand[num1][num0],capacity=cap,setupCost=0)
   # calculate_potential_path(graph.edges)
    nx.draw(graph)
    return graph


def calculate_potential_path(graph, edges,budget):
    for city1, city2, attr in edges:
        if attr['d']>attr['capacity']:
            lack_of_cap=attr['d']-attr['capacity']
            print lack_of_cap
            pot_best_path=graph.add_edge(city1,city2,capacity=cap+lack_of_cap)
            budget -= calculate_cost(graph,city1,city2)
            print budget
        else:
            print "Link is able to handle the demand"
#            pot_best_path=...
#            print "Problem unsolvable"
#        return e.cost

# def set_limitations():
#     # adds limitations to the model, ensures that the requirements for the system are fulfilled
#     u_max = 9 #max 9 modules per link are allowed
#     for v in graph.nodes:
#         print graph.edges.
#         sum(graph.edges)

if __name__ == "__main__":
    graph = draw_graph('polska.xml', cap)
    print graph.edges(data=True)
    calculate_potential_path(graph,graph.edges(data=True),budget)
#    calculate_cost(graph,"Krakow","Bialystok")
#    plt.show()
    # print graph.edges('Bydgoszcz', 'Kolobrzeg')
