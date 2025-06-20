# Fixed Outline Floorplanning
I use B*-tree datastructure with simulated annealing (SA) to optimize wirelength and area.
After SA, I shift the entire block pack within the fixed outline to optimize the wirelength between
blocks and fixed pins. \
Details see `prog2_floorplanning-1.pdf` and `report.pdf`.

# Compile my code:
```bash
make
```
The binary will be generated under `bin/`

# Execute my code:
Enter the `bin/` directory and type the command: 
```bash
./fp <alpha value> <block file path> <nets file path> <output file path>
```

# GUI feature:
Enter the `GUI/` directory. \
Type the command to show the blocks and terminals:
```bash
python3 plot.py -blk <block file path> -rpt <output report> --terminal
```

Type the command to show the blocks, terminals, and wires:
```bash
python3 plot.py -blk <block file path> -rpt <output report> -net <nets file path> --terminal --showwire
```

The python file will generate a figure, `floorplan_plot.png`, under the `GUI/` directory
