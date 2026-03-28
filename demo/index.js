var Module = {
    onRuntimeInitialized: () => {
        Module.ccall(
            'hello_world', // fn name
            null, // return type 
            [], // arg types
            [] // arg vals
        );
    }
};