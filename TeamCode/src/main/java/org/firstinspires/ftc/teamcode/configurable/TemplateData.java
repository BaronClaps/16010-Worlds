package org.firstinspires.ftc.teamcode.configurable;

import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

public class TemplateData {
    public final Map<String, Method> steps = new LinkedHashMap<>();
    public final List<Method> variants = new ArrayList<>();
}
