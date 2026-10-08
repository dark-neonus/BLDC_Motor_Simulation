import type { SidebarsConfig } from "@docusaurus/plugin-content-docs";

const sidebars: SidebarsConfig = {
  docs: [
    "index",
    {
      type: "category",
      label: "Physics & math",
      items: [
        "physics/conventions",
        "physics/motor",
        "physics/mechanical",
        "physics/thermal",
        "physics/inverter",
        "physics/supply",
        "physics/sensors",
        "physics/control",
        "physics/energy",
        "physics/numerics",
        "physics/signals",
        "physics/estimation",
        "physics/validation-catalog",
        "physics/references",
      ],
    },
  ],
};

export default sidebars;
