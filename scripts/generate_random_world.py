import yaml
from world_generator import worldGenerator
import os
from pathlib import Path

if __name__ == "__main__":
    curr_path = os.path.dirname(os.path.abspath(__file__))
    cfg_path = os.path.join(curr_path, "world_generator.yaml")
    cfg = yaml.safe_load(Path(cfg_path).read_text())
    generator = worldGenerator(cfg)
    generator.write_world_file()