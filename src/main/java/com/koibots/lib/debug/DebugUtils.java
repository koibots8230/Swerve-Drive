package com.koibots.lib.debug;

public class DebugUtils {
    public static void printCallerMethodName() {
        System.out.println(
                StackWalker.getInstance().walk(
                        stream -> stream.skip(1).findFirst().get()
                        ).getMethodName()
        );
    }
}
