"""Functions to facilitate message passing/translation and other miscellaneous operations."""


def create_anim_list(robot, path):
    with open(f"{path}/animations.md", "w+") as anim_file:
        anim_file.write("## Animation List\n\n\n* `")
        anim_file.write("\n* `".join(sorted(robot.anim.anim_list)))
        anim_file.write("\n\n## Animation Trigger List\n\n\n* `")
        anim_file.write("`\n* `".join(sorted(robot.anim.anim_trigger_list)))
        anim_file.write("`\n")


def convert_vector3(vector3_obj):
    attr_dict = dict()
    for attr in dir(vector3_obj):
        if attr not in ("x", "y", "z"):
            continue
        attr_dict[attr] = getattr(vector3_obj, attr)

    return attr_dict


def populate_message(message, vector_obj):
    for attr in dir(vector_obj):
        if not hasattr(message, attr):
            continue
        if "__" in attr:
            continue

        val = getattr(vector_obj, attr)
        if val is None:
            val = 0.0

        setattr(message, attr, val)

    return message
