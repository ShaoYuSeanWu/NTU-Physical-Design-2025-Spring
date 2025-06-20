import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from shapely.geometry import box
from shapely.ops import unary_union
import argparse
import sys
import networkx as nx


# Usage: python plot_floorplan.py -rpt <floorplan report> -blk <.block file> [ --terminal [-net <.nets file> --showwire] ] [-tree <bSTree file>] [--show]

def parse_args():
    parser = argparse.ArgumentParser(
        description='Plot floorplan with overlaps and B*-tree visualization.',
        epilog='Commnand line arguments: -rpt <floorplan report> -blk <.block file> [ --terminal [-net <.nets file> --showwire] ] [-tree <bSTree file>] [--show]'
    )
    parser.add_argument('-rpt', type=str, default=None, help='Path to the floorplan report file (must provide)')
    parser.add_argument('-blk', type=str, default=None, help='Path to the .block file (must provide)')
    parser.add_argument('-tree', type=str, default=None, help='Path to the bSTree file (optional)')
    parser.add_argument('-net', type=str, default=None, help='Path to the .nets file (optional)')
    parser.add_argument('--showwire', action='store_true', help='plot wires out (optional)')
    parser.add_argument('--terminal', action='store_true', help='plot terminal out (optional)')
    parser.add_argument('--show', action='store_true', help='Show the plot (optional)')
    parser.print_help()
    return parser.parse_args()


def read_report_file(filename):
    with open(filename, 'r') as f:
        lines = f.readlines()
    data = []
    maxX, maxY = 0, 0
    for line in lines[1:]:
        parts = line.strip().split()
        if (len(parts) == 2):
            maxX, maxY = map(int, parts)
        if len(parts) == 5:
            name = parts[0]
            x1, y1, x2, y2 = map(int, parts[1:])
            data.append((name, x1, y1, x2, y2))
    return maxX, maxY, data

def read_block_file(filename):
    outline_width, outline_height = 0, 0
    with open(filename, 'r') as f:
        lines = f.readlines()
    outline_width, outline_height = map(int, lines[0].split()[1:])
    data = []
    for line in lines[1:]:
        parts = line.strip().split()
        if len(parts) == 4 and parts[1] == 'terminal':
            name = parts[0]
            x, y = map(int, parts[2:])
            data.append((name, x, y))
    return outline_width, outline_height, data

def read_nets_file(filename):
    with open(filename, 'r') as f:
        lines = f.readlines()
    data = []
    netnum = int(lines[0].strip().split()[1])
    linecnt = 1
    for _ in range(netnum):
        line = lines[linecnt]
        linecnt = linecnt + 1
        parts = line.strip().split()
        blocknum = int(parts[1])
        single_net = []
        for i in range(blocknum):
            line = lines[linecnt]
            linecnt = linecnt + 1
            parts = line.strip().split()
            single_net.append(parts[0])
        data.append(single_net)
    return data



def draw_blocks_with_overlaps(report_file, block_file, print_terminal, args):
    outline_width, outline_height, terminals = read_block_file(block_file)
    maxX, maxY, data = read_report_file(report_file)
    if args.net:
        net_data = read_nets_file(args.net)
        

    print ("outline_width, outline_height", outline_width, outline_height)
    print ("maxX, maxY", maxX, maxY)
    
    # adjust maxX and maxY
    if (print_terminal):
        for name, x, y in terminals:
            if (x > maxX):
                maxX = x
            if (y > maxY):
                maxY = y

    # Create the figure and axis
    fig, ax = plt.subplots()
    ax.set_xlim(0, max(maxX, outline_width))
    ax.set_ylim(0, max(maxY, outline_height))

    # Draw the outline
    outline = Rectangle((0, 0), outline_width, outline_height, 
                        linewidth=1.5, edgecolor='black', linestyle='--', facecolor='none')
    ax.add_patch(outline)

    location = {}
    for name, x, y in terminals:
        location[name] = (x, y)
    
    rectangles = []
    for name, x1, y1, x2, y2 in data:
        rect = box(x1, y1, x2, y2)
        rectangles.append(rect)
        ax.add_patch(Rectangle((x1, y1), x2 - x1, y2 - y1, 
                               linewidth=1, edgecolor='#1f77b4', facecolor='#c6dbef'))
        # Add block name at the center
        cx = (x1 + x2) / 2
        cy = (y1 + y2) / 2
        location[name] = (cx, cy)
        # Adjust font size: proportional to the rectangle size
        # font_size = min(x2 - x1, y2 - y1) * 0.2
        # font_size = max(5, min(font_size, 12))  # clamp between 5 and 12
        font_size = 4
        ax.text(cx, cy, name, fontsize=font_size, ha='center', va='center', color='black')

    # Draw terminals
    if (print_terminal):
        for name, x, y in terminals:
            ax.plot(x, y, marker='o', color='#ff7f0e', markersize=2.5, markerfacecolor='none')
            # Add terminal name at the center
            ax.text(x, y, name, fontsize=4, ha='center', va='center', color='black')
    
    # draw wires, use minimum spanning tree to connect a net
    if (args.net and args.showwire):
        for net in net_data:
            G = nx.Graph()
            for i in range(len(net) - 1):
                block1 = net[i]
                for j in range(i + 1, len(net)):
                    block2 = net[j]
                    if (block1 in location and block2 in location):
                        cx1, cy1 = location[block1]
                        cx2, cy2 = location[block2]
                        distance = abs(cx1 - cx2) + abs(cy1 - cy2)
                        G.add_edge(block1, block2, weight=distance)
            T = nx.minimum_spanning_tree(G)
            # draw the edges of the minimum spanning tree
            for u, v in T.edges():
                cx1, cy1 = location[u]
                cx2, cy2 = location[v]
                ax.plot([cx1, cx2], [cy1, cy2], color='#1f77b4', linewidth=0.5, alpha=0.7)
                # Add wire name at the center
                # cx = (cx1 + cx2) / 2
                # cy = (cy1 + cy2) / 2
                # ax.text(cx, cy, f"{u}-{v}", fontsize=4, ha='center', va='center', color='black')



    # Highlight overlaps
    for i in range(len(rectangles)):
        for j in range(i + 1, len(rectangles)):
            inter = rectangles[i].intersection(rectangles[j])
            if not inter.is_empty:
                if inter.geom_type == 'Polygon':
                    x, y = inter.exterior.xy
                    ax.fill(x, y, color='#d62728', alpha=0.5)

    ax.set_aspect('equal')
    # plt.gca().invert_yaxis()  # Optional: flip y-axis
    plt.title("Block Layout")

    # for better GUI
    ax.set_aspect('equal')
    ax.set_title("Block Layout", fontsize=12, fontweight='bold')
    ax.set_xlabel("X-coordinate")
    ax.set_ylabel("Y-coordinate")
    ax.grid(True, linestyle='--', linewidth=0.3)

    # Add legend for visuals
    from matplotlib.lines import Line2D
    legend_elements = [
        Line2D([0], [0], color='#1f77b4', lw=1, label='Block'),
        Line2D([0], [0], color='#d62728', lw=6, alpha=0.5, label='Overlap'),
        Line2D([0], [0], marker='o', color='#ff7f0e', label='Terminal', markerfacecolor='none', markersize=5),
    ]

    if (args.net and args.showwire):
        legend_elements.append(Line2D([0], [0], color='#1f77b4', lw=0.5, alpha=0.7, label='Wire'))

    ax.legend(handles=legend_elements, loc='upper right', fontsize=6)

    plt.tight_layout()
    fig.canvas.manager.set_window_title('Floorplan Viewer')

    # save the figure
    plt.savefig('floorplan_plot.png', dpi=300, bbox_inches='tight')
    # plt.show()

def draw_blocks_with_tree(report_file, block_file, treefile):
    outline_width, outline_height, terminals = read_block_file(block_file)
    maxX, maxY, data = read_report_file(report_file)
    
    # Create the figure and axis
    fig, ax = plt.subplots()
    ax.set_xlim(0, max(maxX, outline_width))
    ax.set_ylim(0, max(maxY, outline_height))

    location = {}

    # Draw the outline
    outline = Rectangle((0, 0), outline_width, outline_height, 
                        linewidth=1.5, edgecolor='black', linestyle='--', facecolor='none')
    ax.add_patch(outline)

    rectangles = []
    for name, x1, y1, x2, y2 in data:
        rect = box(x1, y1, x2, y2)
        rectangles.append(rect)
        ax.add_patch(Rectangle((x1, y1), x2 - x1, y2 - y1, 
                               linewidth=1, edgecolor='#1f77b4', facecolor='#c6dbef'))
        # Add block name at the center
        cx = (x1 + x2) / 2
        cy = (y1 + y2) / 2
        location[name] = (cx, cy)

        # Draw dark green dot at the center
        ax.plot(cx, cy, marker='o', color='#2ca02c', markersize=2.5, markerfacecolor='none')

    # Read the tree file and plot the tree
    with open(treefile, 'r') as f:
        lines = f.readlines()
    for line in lines:
        parts = line.strip().split()
        if (parts[0] == 'root:'):
            for i in range(1, len(parts)-1):
                name1 = parts[i]
                name2 = parts[i+1]
                if (name1 in location and name2 in location):
                    cx1, cy1 = location[name1]
                    cx2, cy2 = location[name2]
                    # Draw a line between the two blocks
                    ax.plot([cx1, cx2], [cy1, cy2], color='#2ca02c', linewidth=0.5)
        else:
            parts[0] = parts[0].split(':')[0]
            for i in range(0, len(parts)-1):
                name1 = parts[i]
                name2 = parts[i+1]
                if (name1 in location and name2 in location):
                    cx1, cy1 = location[name1]
                    cx2, cy2 = location[name2]
                    # Draw a line between the two blocks
                    ax.plot([cx1, cx2], [cy1, cy2], color='#2ca02c', linewidth=0.5)
                

    # Highlight overlaps
    for i in range(len(rectangles)):
        for j in range(i + 1, len(rectangles)):
            inter = rectangles[i].intersection(rectangles[j])
            if not inter.is_empty:
                if inter.geom_type == 'Polygon':
                    x, y = inter.exterior.xy
                    ax.fill(x, y, color='#d62728', alpha=0.5)

    ax.set_aspect('equal')
    # plt.gca().invert_yaxis()  # Optional: flip y-axis
    plt.title("Block Layout")

    # for better GUI
    ax.set_aspect('equal')
    ax.set_title("Block Layout", fontsize=12, fontweight='bold')
    ax.set_xlabel("X-coordinate")
    ax.set_ylabel("Y-coordinate")
    ax.grid(True, linestyle='--', linewidth=0.3)

    # Add legend for visuals
    from matplotlib.lines import Line2D
    legend_elements = [
        Line2D([0], [0], color='#1f77b4', lw=1, label='Block'),
        Line2D([0], [0], color='#d62728', lw=6, alpha=0.5, label='Overlap'),
        Line2D([0], [0], marker='o', color='#2ca02c', label='B*-tree', markerfacecolor='none', markersize=5),
    ]

    ax.legend(handles=legend_elements, loc='upper right', fontsize=6)

    plt.tight_layout()
    fig.canvas.manager.set_window_title('Floorplan Viewer')

    # save the figure
    plt.savefig('floorplan_tree_plot.png', dpi=300, bbox_inches='tight')
    # plt.show()

if __name__ == "__main__":
    args = parse_args()
    report_file = args.rpt
    block_file = args.blk
    tree_file = args.tree
    if (not report_file or not block_file):
        print("Error: Must provide both -rpt and -blk files.")
        sys.exit(1)
    
    if (args.terminal):
        print("Drawing blocks and terminal with overlaps...")
    else:
        print("Drawing blocks with overlaps...")

    draw_blocks_with_overlaps(report_file, block_file, args.terminal, args)
    
    if (args.tree):
        print("Drawing blocks with B*-tree...")
        draw_blocks_with_tree(report_file, block_file, tree_file)
    
    if (args.show):
        print("Showing the plot...")
        plt.show()
