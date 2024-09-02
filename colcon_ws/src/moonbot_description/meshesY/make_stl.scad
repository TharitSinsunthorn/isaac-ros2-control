module centered_cylinder() {
    // 円柱の中央を原点に配置
    translate([0, 0, -75])
        cylinder(h=150, r=60);
}

// 全体の形状を描画
centered_cylinder();
