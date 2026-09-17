# Contributing — New Assets

This guide explains how to contribute new assets to the Space Robotics Bench.

## Asset Types Overview

The Space Robotics Bench categorizes assets into three main types:

| Asset Type  | Description                                        |
| ----------- | -------------------------------------------------- |
| **Scenery** | Environmental elements like terrain and structures |
| **Objects** | Interactive objects, tools, and payloads           |
| **Robots**  | Systems that are intended to be controlled         |

## Static Assets

All static assets used by the Space Robotics Bench are separated into the [srb_assets](https://github.com/AndrejOrsula/srb_assets) repository to encourage their reuse.

If you wish to contribute your asset, please follow these guidelines:

- Simplify the mesh for efficient simulation
- Aim for watertight meshes with clean topology
- Bake materials into PBR textures for compatibility
- Export meshes in the USD format (`.usd`/`.usda`/`.usdc`/`.usdz`)
- For articulated assets, add relevant joints and APIs (you can use Isaac Sim for this)

Afterward, you can add the asset to your [fork of srb_assets](https://github.com/AndrejOrsula/srb_assets/fork) and submit a pull request. We greatly appreciate your contributions!

## Procedural Assets with SimForge

> Reference: [SimForge](https://AndrejOrsula.github.io/simforge)

[SimForge](https://github.com/AndrejOrsula/simforge) is a framework for creating diverse virtual environments through procedural generation. SRB leverages SimForge to generate all procedural assets. Below are some examples:

<iframe style="width:100%;aspect-ratio:16/9" src="https://www.youtube.com/embed/XgXYHmEIvSM?si=IZGp8lwmlxWIsO4h&mute=1&autoplay=1&loop=1&playlist=XgXYHmEIvSM" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

<iframe style="width:100%;aspect-ratio:16/9" src="https://www.youtube.com/embed/jd7IIaL5Vgg?si=yCifBNo3eu6J22Vy&mute=1&autoplay=1&loop=1&playlist=jd7IIaL5Vgg" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

<iframe style="width:100%;aspect-ratio:16/9" src="https://www.youtube.com/embed/DbnE086w-sY?si=WXsuxjkrSW6Oxy1A&mute=1&autoplay=1&loop=1&playlist=DbnE086w-sY" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

Please refer to the [SimForge documentation](https://AndrejOrsula.github.io/simforge) if you wish to contribute new procedural assets, particularly the following sections:

- [New Assets](https://AndrejOrsula.github.io/simforge/development/assets.html)
  - [Blender Generator: New Assets](https://andrejorsula.github.io/simforge/generators/blender.html#new-assets)

We look forward to seeing your fantastic contributions. Many thanks in advance!
