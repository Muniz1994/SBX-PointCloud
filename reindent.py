def reindent(path):
    with open(path, encoding="utf-8") as f:
        lines = f.read().split("\n")
    out = []
    level = 0
    for raw in lines:
        s = raw.rstrip()
        if s == "":
            out.append("")
            continue
        if s.startswith("else:") or s.startswith("elif "):
            if level > 0:
                level -= 1
        out.append("\t" * level + s)
        code = s.split("#")[0].rstrip()
        if code.endswith(":"):
            level += 1
    with open(path, "w", newline="\n", encoding="utf-8") as f:
        f.write("\n".join(out) + "\n")

reindent("addons/point_cloud/point_cloud_dock.gd")
print("Done")