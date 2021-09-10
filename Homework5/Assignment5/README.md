### 完成的点

- 提交的格式正确，包含所有必须的文件。代码可以编译和运行
- 光线生成：
  正确实现光线生成部分，并且能够看到图像中的两个球体
- 光线与三角形相交：
  正确实现了Moller-Trumbore 算法，并且能够看到图像中的地面。

scene中的场景被压缩成[-1,1]，需要还原成[0, width]和[0, height]

```cpp
for (int j = 0; j < scene.height; ++j)
    {
        for (int i = 0; i < scene.width; ++i)
        {
            // generate primary ray direction
            float x;
            float y;
            // TODO: Find the x and y positions of the current pixel to get the direction
            // vector that passes through it.
            // Also, don't forget to multiply both of them with the variable *scale*, and
            // x (horizontal) variable with the *imageAspectRatio*      

            x = (((i + 0.5) / ((float)scene.width) * 2) - 1) * imageAspectRatio * scale;
            y = (1 - (j + 0.5) / (float)scene.height * 2) * scale;

            Vector3f dir = Vector3f(x, y, -1); // Don't forget to normalize this direction!
            dir = normalize(dir);
            framebuffer[m++] = castRay(eye_pos, dir, scene, 0);
        }
        UpdateProgress(j / (float)scene.height);
    }
```

实现**Möller-Trumbore**射线三角形求交算法，因(1-u-v)>=0会使场景中地面某几点显示成蓝色，经Debug在该点正处于射线与三角形交点的边界处，浮点运算趋向于0 ，于是用-FLT_EPSILON替代

```cpp
bool rayTriangleIntersect(const Vector3f& v0, const Vector3f& v1, const Vector3f& v2, const Vector3f& orig,
                          const Vector3f& dir, float& tnear, float& u, float& v)
{
    // TODO: Implement this function that tests whether the triangle
    // that's specified bt v0, v1 and v2 intersects with the ray (whose
    // origin is *orig* and direction is *dir*)
    // Also don't forget to update tnear, u and v.
    bool isIn = false;
    Vector3f E1 = v1 - v0;
    Vector3f E2 = v2 - v0;
    Vector3f S = orig - v0;

    Vector3f S1 = crossProduct(dir, E2);
    Vector3f S2 = crossProduct(S, E1);

    float S1E1Dot = dotProduct(S1, E1);
    float S2E2Dot = dotProduct(S2, E2);
    float S1SDot = dotProduct(S1, S);
    float S2DirDot = dotProduct(S2, dir);

    // 共同系数
    float coeff = 1.0f / S1E1Dot;
    tnear = coeff * S2E2Dot;
    u = coeff * S1SDot;
    v = coeff * S2DirDot;

	if (tnear >= 0 && u >= 0 && v >= 0 && (1 - u - v) >= -FLT_EPSILON)
	{
		isIn = true;
	}

    return isIn;
}
```

