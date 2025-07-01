import pandas as pd
import networkx as nx
import matplotlib.pyplot as plt

def plot_directed_graph_with_edges(nodes_file, edges_file):
    # Load data points
    nodes_df = pd.read_csv(nodes_file)
    edges_df = pd.read_csv(edges_file)

    # Create directed graph
    G = nx.DiGraph()
    for _, row in nodes_df.iterrows():
        G.add_node(row['id'], pos=(row['lon'], row['lat']))

    for _, row in edges_df.iterrows():
        G.add_edge(row['source'], row['target'],
                   id=row['id'],
                   on_shortest_path=row['on_shortest_path'],
                   shortcut=row['shortcut'])

    pos = nx.get_node_attributes(G, 'pos')

    # Prepare edge colors
    edge_colors = []
    for u, v in G.edges():
        if G[u][v]['on_shortest_path']:
            edge_colors.append('red')
        elif G[u][v]['shortcut']:
            edge_colors.append('lavender')
        else:
            edge_colors.append('gray')

    plt.figure(figsize=(10, 8))

    # Draw nodes
    nx.draw_networkx_nodes(G, pos, node_color='skyblue', node_size=500)
    nx.draw_networkx_labels(G, pos)

    # To handle bidirectional edges, find edges that appear both ways
    bidirectional_edges = set()
    for u, v in G.edges():
        if G.has_edge(v, u):
            # store ordered tuple to avoid duplicates
            bidirectional_edges.add(tuple(sorted((u, v))))

    # Draw edges with arrows
    for idx, (u, v) in enumerate(G.edges()):
        color = edge_colors[idx]
        if tuple(sorted((u, v))) in bidirectional_edges:
            # Draw a curved arrow for bidirectional edges
            # Check if already drawn the reverse edge curved arrow to avoid duplicates
            if (v, u) in G.edges() and u < v:
                # Draw curved arrows both ways
                nx.draw_networkx_edges(
                    G, pos,
                    edgelist=[(u, v)],
                    connectionstyle='arc3,rad=0.2',
                    edge_color=color,
                    arrowsize=20,
                    arrowstyle='-|>',
                    width=2
                )
                nx.draw_networkx_edges(
                    G, pos,
                    edgelist=[(v, u)],
                    connectionstyle='arc3,rad=-0.2',
                    edge_color=edge_colors[list(G.edges()).index((v, u))],
                    arrowsize=20,
                    arrowstyle='-|>',
                    width=2
                )
        else:
            # Single direction edge, draw straight arrow
            nx.draw_networkx_edges(
                G, pos,
                edgelist=[(u, v)],
                edge_color=color,
                arrowsize=20,
                arrowstyle='-|>',
                width=2
            )

    # Draw edge labels (IDs)
    edge_labels = {(u, v): G[u][v]['id'] for u, v in G.edges()}
    nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels, font_color='blue')

    plt.xlabel("Longitude")
    plt.ylabel("Latitude")
    plt.title(f"Directed Graph from {nodes_file} and {edges_file}\nEdge IDs, Shortest Paths (red), Shortcuts (lavender)")
    plt.axis('off')
    # plt.grid(True)
    plt.show()


# Example usage
plot_directed_graph_with_edges('nodes.csv', 'edges.csv')
plot_directed_graph_with_edges('nodes.csv', 'downward_edges.csv')
plot_directed_graph_with_edges('nodes.csv', 'upward_edges.csv')
