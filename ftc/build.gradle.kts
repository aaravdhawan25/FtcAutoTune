plugins {
    id("com.android.library")
    `maven-publish`
}

android {
    namespace = "com.aaravdhawan25.pidautotuner.ftc"
    compileSdk = 30

    defaultConfig {
        minSdk = 24
    }

    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_1_8
        targetCompatibility = JavaVersion.VERSION_1_8
    }

    publishing {
        singleVariant("release") {
            withSourcesJar()
        }
    }
}

dependencies {
    // The math/algorithm classes are exposed transitively to the TeamCode module.
    api(project(":core"))

    // The FTC SDK is provided by the host (TeamCode/FtcRobotController) app at
    // runtime, so it is only needed here at compile time. Match these to
    // whatever SDK version your season's Quickstart uses.
    compileOnly("org.firstinspires.ftc:RobotCore:11.1.0")
    compileOnly("org.firstinspires.ftc:Hardware:11.1.0")
    compileOnly("org.firstinspires.ftc:FtcCommon:11.1.0")

    // Optional: uncomment if you wire up FTC Dashboard graphing in the OpModes.
    // compileOnly("com.acmerobotics.dashboard:dashboard:0.4.16")
}

publishing {
    publications {
        register<MavenPublication>("release") {
            groupId = project.group.toString()
            artifactId = "pidautotuner-ftc"
            version = project.version.toString()

            afterEvaluate {
                from(components["release"])
            }
        }
    }
}
