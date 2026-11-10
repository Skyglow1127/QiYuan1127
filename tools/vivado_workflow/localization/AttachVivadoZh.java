import com.sun.tools.attach.VirtualMachine;

/** Attach only to the exact Vivado PID supplied by the Ross launch receipt. */
public final class AttachVivadoZh {
    public static void main(String[] args) throws Exception {
        if(args.length!=3) throw new IllegalArgumentException("PID agent.jar arguments");
        VirtualMachine vm=VirtualMachine.attach(args[0]);
        try { vm.loadAgent(args[1],args[2]); }
        finally {vm.detach();}
    }
}
