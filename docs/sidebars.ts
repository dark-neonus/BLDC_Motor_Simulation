import type { SidebarsConfig } from "@docusaurus/plugin-content-docs";

const sidebars: SidebarsConfig = {
  docs: [
    "index",
    {
      type: "category",
      label: "Physics & math",
      items: ["physics/conventions", "physics/references"],
    },
  ],
};

export default sidebars;
